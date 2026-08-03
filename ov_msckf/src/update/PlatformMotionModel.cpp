#include "PlatformMotionModel.h"

#include <cmath>
#include <algorithm>

#include "utils/print.h"

using namespace ov_msckf;

// ============================================================================
// base_noise
//
// Continuous-time noise covariance, per axis, from the platform profile.
// Same discretization convention as the original UpdaterWheel
// (sigma^2 / dt on the diagonal, consumed by the G*Q*G^T dt^2 propagation).
// ============================================================================
Eigen::Matrix<double, 6, 6> PlatformMotionModel::base_noise(double dt) const {

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();

    // Platform knowledge enters here as per-axis TIGHTENING of the constrained
    // axes, not as inflation. The odometry stream reports v = [v_x, 0, 0]; the
    // zeros on y and z are not dead channels, they are the nonholonomic
    // constraint itself, asserted at every sample and converted by the wheel
    // preintegration into a relative displacement between clones, which is the
    // correct temporal model for a standing kinematic fact (counted once per
    // clone interval, noise integrated over the interval, FEJ-consistent
    // Jacobians in compute_linear_system). A small sigma on y/z states how far
    // the true body velocity may depart from zero (tyre slip, suspension
    // motion), so morphology is expressed entirely through noise_v_axis.
    // The previous version inflated constrained axes to 10x the quietest axis
    // on the premise that those channels are noise; that removed exactly the
    // constraint information the wheel channel was carrying, measured on
    // urban39 as 8.915 m -> 12.951 m APE with the explicit constraint off.
    for (int i = 0; i < 3; i++) {
        const double sw = _opts.noise_w_axis(i);
        const double sv = _opts.noise_v_axis(i);
        Q(i, i)         = (sw * sw) / dt;   // rotation block
        Q(3 + i, 3 + i) = (sv * sv) / dt;   // position block
    }
    return Q;
}

// ============================================================================
// push_innovation
// ============================================================================
void PlatformMotionModel::push_innovation(double timestamp,
                                          const Eigen::Matrix<double, 6, 1> &innov) {

    _time_buf.push_back(timestamp);
    for (int i = 0; i < 6; i++) {
        _raw_buf[i].push_back(innov(i));
    }
    while ((int)_time_buf.size() > _opts.gait_window) {
        _time_buf.pop_front();
        for (int i = 0; i < 6; i++) _raw_buf[i].pop_front();
    }
}

// ============================================================================
// detect_periodicity
//
// Normalized autocorrelation. A noise sequence decorrelates immediately;
// a gait oscillation produces a strong peak at lag = 1/f_gait. We accept the
// peak only if the implied frequency lies inside the configured gait band,
// which prevents slow drifts or aliasing from being classified as gait.
// ============================================================================
bool PlatformMotionModel::detect_periodicity(const std::deque<double> &buf,
                                             double mean_dt,
                                             double &freq_out,
                                             double &power_out) const {

    const int n = (int)buf.size();
    if (n < 32 || mean_dt <= 0.0) return false;

    // De-mean
    double mean = 0.0;
    for (double v : buf) mean += v;
    mean /= n;

    std::vector<double> x(n);
    double var = 0.0;
    for (int i = 0; i < n; i++) {
        x[i] = buf[i] - mean;
        var += x[i] * x[i];
    }
    var /= n;
    if (var < 1e-12) return false;

    // Lag range implied by the gait frequency band
    int lag_min = std::max(2, (int)std::floor(1.0 / (_opts.gait_freq_max * mean_dt)));
    int lag_max = std::min(n / 2, (int)std::ceil(1.0 / (_opts.gait_freq_min * mean_dt)));
    if (lag_min >= lag_max) return false;

    double best_r = 0.0;
    int best_lag = -1;
    for (int lag = lag_min; lag <= lag_max; lag++) {
        double acc = 0.0;
        for (int i = 0; i + lag < n; i++) acc += x[i] * x[i + lag];
        double r = acc / ((n - lag) * var);   // normalized autocorrelation
        if (r > best_r) { best_r = r; best_lag = lag; }
    }

    if (best_lag < 0 || best_r < _opts.gait_periodicity_thresh) return false;

    // Oscillation signature check: a genuine gait oscillation (sinusoid-like)
    // is strongly ANTI-correlated at half its period, while a periodic
    // impulse train (repeating glitches, encoder faults) is not. This keeps
    // impulsive artifacts on the correntropy down-weighting path.
    const int half_lag = best_lag / 2;
    if (half_lag >= 2) {
        double acc_h = 0.0;
        for (int i = 0; i + half_lag < n; i++) acc_h += x[i] * x[i + half_lag];
        double r_half = acc_h / ((n - half_lag) * var);
        if (r_half > -0.25 * best_r) return false;
    }

    freq_out  = 1.0 / (best_lag * mean_dt);
    power_out = best_r * var;   // variance share of the periodic component
    return true;
}

// ============================================================================
// adapt_constraint_R
// ============================================================================
void PlatformMotionModel::adapt_constraint_R(Eigen::MatrixXd &R,
                                             const Eigen::VectorXd &res,
                                             const std::vector<int> &axes) {

    if (!_opts.do_constraint_adaptive)
        return;

    const double s = std::max(_opts.constraint_corr_sigma, 1e-6);

    for (int k = 0; k < (int)axes.size(); k++) {
        const double r_kk = std::max(R(k, k), 1e-12);
        const double n = std::fabs(res(k)) / std::sqrt(r_kk);   // normalized innovation

        double g = std::exp(-(n * n) / (2.0 * s * s));
        g = std::max(g, _opts.constraint_corr_gain_min);

        R(k, k) *= 1.0 / (g * g);
        _constraint_gain[axes[k]] = g;
    }
}

// ============================================================================
// adapt_R
// ============================================================================
void PlatformMotionModel::adapt_R(Eigen::Matrix<double, 6, 6> &R,
                                  const Eigen::Matrix<double, 6, 1> &res) {

    // Off = the wheel update is exactly the legacy path except for the
    // per-axis Q from base_noise. The urban39 ablations need this isolation:
    // adapt_R alone moved the wheel result 8.915 m -> 10.786 m, so leaving it
    // active would mix that penalty into the noise-shaping measurement.
    if (!_opts.do_wheel_adaptive)
        return;

    // ---- 1) normalized innovation per axis (adaptive-branch formula) ----
    Eigen::Matrix<double, 6, 1> n_innov;
    for (int i = 0; i < 6; i++) {
        double r_ii = std::max(R(i, i), 1e-12);
        n_innov(i) = std::fabs(res(i)) / std::sqrt(r_ii);
    }

    // ---- 2) refresh periodicity classification ----
    double mean_dt = 0.0;
    if (_time_buf.size() >= 2) {
        mean_dt = (_time_buf.back() - _time_buf.front()) / (double)(_time_buf.size() - 1);
    }
    if (_opts.do_gait_model && mean_dt > 0.0) {
        for (int i = 0; i < 6; i++) {
            double f = 0.0, p = 0.0;
            _axes[i].periodic = detect_periodicity(_raw_buf[i], mean_dt, f, p);
            _axes[i].freq_hz  = _axes[i].periodic ? f : 0.0;
            _axes[i].osc_var  = _axes[i].periodic ? p : 0.0;
        }
    }

    // Window not full yet: identity behaviour, exactly like the adaptive branch
    const bool window_full = active();

    // Per-axis multiplicative factors. They are collected here and applied once
    // at the end as a congruence transform: R is not diagonal (the -skew(dp)
    // block of the preintegration Jacobian couples rotation into position), so
    // touching only R(i,i) would leave the cross-terms behind and any factor
    // below 1 could push R indefinite.
    Eigen::Matrix<double, 6, 1> axis_scale = Eigen::Matrix<double, 6, 1>::Ones();

    for (int i = 0; i < 6; i++) {

        if (_opts.do_gait_model && _axes[i].periodic) {
            // STRUCTURED MOTION: gait oscillation is real. Do NOT apply the
            // correntropy penalty on this axis. Absorb the oscillation power
            // into the expected variance so chi2 and the gain remain
            // consistent, instead of the filter fighting the gait.
            R(i, i) += _axes[i].osc_var;   // additive on the diagonal is PSD-safe
            _axes[i].corr_gain = 1.0;

        } else if (window_full) {
            // NOISE PATH: per-axis correntropy weight applied via R scaling,
            // same mechanism as the adaptive branch's correntropy gain (R *= 1/G^2).
            double s = std::max(_opts.corr_sigma(i), 1e-6);
            double g = std::exp(-(n_innov(i) * n_innov(i)) / (2.0 * s * s));
            g = std::max(g, 1e-3);
            axis_scale(i) *= 1.0 / (g * g);
            _axes[i].corr_gain = g;
        }
    }

    // ---- 3) per-axis sliding-window R scale (adaptive-branch estimate_R) ----
    if (window_full) {
        const int n_buf = (int)_innov_window.size();
        const int start = std::max(0, n_buf - _opts.corr_recent);
        Eigen::Matrix<double, 6, 1> sum_sq = Eigen::Matrix<double, 6, 1>::Zero();
        int count = 0;
        for (int k = start; k < n_buf; k++) {
            sum_sq += _innov_window[k].cwiseProduct(_innov_window[k]);
            count++;
        }
        if (count > 0) {
            for (int i = 0; i < 6; i++) {
                // Gait axes already model their variance explicitly; scaling
                // them again would double-count the oscillation.
                if (_opts.do_gait_model && _axes[i].periodic) {
                    _axes[i].r_scale = 1.0;
                    continue;
                }
                double mean_sq = sum_sq(i) / count;
                double scale = std::min(std::max(mean_sq, _opts.r_scale_min),
                                        _opts.r_scale_max);
                axis_scale(i) *= scale;
                _axes[i].r_scale = scale;
            }
        }
    }

    // ---- 4) apply the accumulated per-axis factors as a congruence ----
    // R <- D R D with D = diag(sqrt(scale)), i.e. R_ij *= sqrt(s_i * s_j).
    // This scales the cross-terms consistently and provably keeps R positive
    // definite for any positive scale, unlike scaling the diagonal alone.
    if (!axis_scale.isOnes()) {
        const Eigen::Matrix<double, 6, 1> d = axis_scale.cwiseSqrt();
        R = R.cwiseProduct(d * d.transpose());
    }

    // ---- 5) numerical guard ----
    // StateHelper::EKFUpdate kills the process when the state covariance comes
    // back non-PSD, so never hand it an R that is not positive definite.
    R = 0.5 * (R + R.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(R);
    if (es.info() == Eigen::Success) {
        Eigen::Matrix<double, 6, 1> ev = es.eigenvalues();
        const double ev_floor = 1e-9 * std::max(1.0, ev.maxCoeff());
        if (ev.minCoeff() < ev_floor) {
            ev = ev.cwiseMax(ev_floor);
            R = es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
        }
    }

    // ---- 6) advance the normalized-innovation window ----
    _innov_window.push_back(n_innov);
    while ((int)_innov_window.size() > _opts.corr_window) {
        _innov_window.pop_front();
    }

    PRINT_DEBUG("[PLATFORM] adapt_R | active=%d gaitZ=%d fZ=%.2fHz gains=[%.2f %.2f %.2f | %.2f %.2f %.2f]\n",
                (int)window_full, (int)_axes[5].periodic, _axes[5].freq_hz,
                _axes[0].corr_gain, _axes[1].corr_gain, _axes[2].corr_gain,
                _axes[3].corr_gain, _axes[4].corr_gain, _axes[5].corr_gain);
}
