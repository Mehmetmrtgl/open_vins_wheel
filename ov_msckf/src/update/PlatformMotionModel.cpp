#include "PlatformMotionModel.h"

#include <algorithm>
#include <cmath>

#include "utils/print.h"

using namespace ov_msckf;

// ============================================================================
// Construction
// ============================================================================
PlatformMotionModel::PlatformMotionModel(const OptionsPlatform &opts)
    : _opts(opts), _corr(opts.corr) {

    for (int i = 0; i < 3; i++) {
        if (_opts.meas_mask(i) == 1)
            _axes_idx.push_back(i);
    }
}

// ============================================================================
// base_R
// ============================================================================
Eigen::MatrixXd PlatformMotionModel::base_R() const {

    const int m = (int)_axes_idx.size();
    const double sb = (_opts.bias_tau > 0.0) ? std::max(_opts.bias_sigma, 0.0) : 0.0;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    for (int k = 0; k < m; k++) {
        const double s = std::max(_opts.meas_sigma(_axes_idx[k]), 1e-4);
        // In quadrature: the residual has been corrected by an estimate, so it
        // cannot be trusted more tightly than that estimate is known.
        R(k, k) = s * s + sb * sb;
    }
    return R;
}

// ============================================================================
// debias
// ============================================================================
Eigen::VectorXd PlatformMotionModel::debias(const Eigen::VectorXd &res_raw) const {

    const int m = (int)_axes_idx.size();
    if (_opts.bias_tau <= 0.0 || (int)res_raw.size() != m)
        return res_raw;

    Eigen::VectorXd res(m);
    for (int k = 0; k < m; k++) {
        res(k) = res_raw(k) - _bias[_axes_idx[k]];
    }
    return res;
}

// ============================================================================
// push_residual
// ============================================================================
void PlatformMotionModel::push_residual(double timestamp,
                                        const Eigen::VectorXd &res) {

    if ((int)res.size() != (int)_axes_idx.size())
        return;

    // ---- slowly varying offset ----
    // First-order low pass on the RAW residual. tau must be long compared with
    // a turn so that genuine lateral motion is not absorbed; at 30 s and the
    // ~3 Hz evaluation rate this averages ~100 samples.
    if (_opts.bias_tau > 0.0) {
        if (_bias_time >= 0.0) {
            const double dt = timestamp - _bias_time;
            if (dt > 0.0) {
                const double alpha = std::min(1.0, dt / _opts.bias_tau);
                for (int k = 0; k < (int)_axes_idx.size(); k++) {
                    double &b = _bias[_axes_idx[k]];
                    b += alpha * (res(k) - b);
                }
            }
        }
        _bias_time = timestamp;
    }

    _time_buf.push_back(timestamp);
    for (int k = 0; k < (int)_axes_idx.size(); k++) {
        _raw_buf[_axes_idx[k]].push_back(res(k));
    }
    while ((int)_time_buf.size() > _opts.gait_window) {
        _time_buf.pop_front();
        for (int a : _axes_idx)
            _raw_buf[a].pop_front();
    }
}

// ============================================================================
// detect_periodicity
//
// Normalized autocorrelation. A noise sequence decorrelates immediately; a gait
// oscillation produces a strong peak at lag = 1/f_gait. The peak is accepted
// only if the implied frequency lies inside the configured gait band, which
// keeps slow drifts and aliasing from being classified as gait.
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
    // is strongly ANTI-correlated at half its period, while a periodic impulse
    // train (repeating glitches, encoder faults) is not. This keeps impulsive
    // artifacts on the correntropy down-weighting path, where they belong.
    const int half_lag = best_lag / 2;
    if (half_lag >= 2) {
        double acc_h = 0.0;
        for (int i = 0; i + half_lag < n; i++) acc_h += x[i] * x[i + half_lag];
        double r_half = acc_h / ((n - half_lag) * var);
        if (r_half > -0.25 * best_r) return false;
    }

    freq_out = 1.0 / (best_lag * mean_dt);
    power_out = best_r * var;   // variance share of the periodic component
    return true;
}

// ============================================================================
// adapt_R
// ============================================================================
void PlatformMotionModel::adapt_R(Eigen::MatrixXd &R, const Eigen::VectorXd &res,
                                  const Eigen::MatrixXd &HPHt) {

    const int m = (int)_axes_idx.size();
    if (m == 0 || (int)res.size() != m || R.rows() != m || R.cols() != m)
        return;

    // ---- 1) refresh the periodicity classification ----
    double mean_dt = 0.0;
    if (_time_buf.size() >= 2) {
        mean_dt = (_time_buf.back() - _time_buf.front()) / (double)(_time_buf.size() - 1);
    }
    if (mean_dt > 0.0) {
        for (int a : _axes_idx) {
            double f = 0.0, p = 0.0;
            _axis[a].periodic = detect_periodicity(_raw_buf[a], mean_dt, f, p);
            _axis[a].freq_hz = _axis[a].periodic ? f : 0.0;
            _axis[a].osc_var = _axis[a].periodic ? p : 0.0;
        }
    }

    // ---- 2) absorb the structured component ----
    // Additive on the diagonal, which is PSD-safe, and it says the right thing:
    // the model expects motion of this magnitude on this axis, so a residual of
    // that size is consistent rather than anomalous.
    std::vector<bool> explained(m, false);
    for (int k = 0; k < m; k++) {
        const int a = _axes_idx[k];
        if (_axis[a].periodic) {
            R(k, k) += _axis[a].osc_var;
            explained[k] = true;
        }
    }

    // ---- 3) correntropy weighting + innovation-based noise estimate ----
    // Gait absorption above is already folded into R, so S below carries it too
    // and a modelled oscillation does not read as an inconsistency.
    _corr.apply(R, res, HPHt, explained);

    for (int k = 0; k < m; k++) {
        const int a = _axes_idx[k];
        _axis[a].gain = _corr.channel(k).gain;
        _axis[a].r_scale = _corr.channel(k).r_scale;
    }

    PRINT_DEBUG("[PLATFORM] adapt_R | active=%d gait=[%d %d %d] f=[%.2f %.2f %.2f]Hz "
                "G=[%.2f %.2f %.2f] rs=[%.2f %.2f %.2f]\n",
                (int)_corr.active(),
                (int)_axis[0].periodic, (int)_axis[1].periodic, (int)_axis[2].periodic,
                _axis[0].freq_hz, _axis[1].freq_hz, _axis[2].freq_hz,
                _axis[0].gain, _axis[1].gain, _axis[2].gain,
                _axis[0].r_scale, _axis[1].r_scale, _axis[2].r_scale);
}
