#include "CorrentropyFilter.h"

#include <algorithm>
#include <cmath>

using namespace ov_msckf;

void CorrentropyFilter::reset() {
    _window.clear();
    _channels.clear();
}

void CorrentropyFilter::apply(Eigen::MatrixXd &R, const Eigen::VectorXd &res,
                              const Eigen::MatrixXd &HPHt,
                              const std::vector<bool> &explained) {

    const int m = (int)res.size();
    if (m == 0 || R.rows() != m || R.cols() != m)
        return;
    if (HPHt.rows() != m || HPHt.cols() != m)
        return;

    // Measurement dimension changed: the stored normalized innovations no
    // longer describe the same channels, so they must not be reused.
    if ((int)_channels.size() != m) {
        _channels.assign(m, Channel());
        _window.clear();
    }

    // ---- 1) normalized innovation per channel ----
    // Against S = H P H^T + R, the covariance the residual actually has, taken
    // before this call adapts R so the weight measures the residual against what
    // the filter predicted rather than against its own correction. mean(n^2) = 1
    // is then exactly the statement that the filter is consistent.
    //
    // Normalizing against R alone was measured as an alternative on urban39 and
    // is within noise of this on that bag (S/R ~ 1.18 there, so n differs by
    // ~9% and the resulting weights are indistinguishable). S is kept because it
    // is the denominator the residual is actually distributed against, and the
    // gap between the two grows exactly when the state is uncertain, which is
    // when the difference matters.
    Eigen::VectorXd n = Eigen::VectorXd::Zero(m);
    for (int i = 0; i < m; i++) {
        const double s_ii = std::max(HPHt(i, i) + R(i, i), 1e-12);
        n(i) = std::fabs(res(i)) / std::sqrt(s_ii);
        _channels[i].norm_innov = n(i);
    }

    const double sigma = std::max(_opts.kernel_sigma, 1e-6);
    const bool window_full = active();

    // ---- 2) per-channel scale: correntropy weight x noise estimate ----
    Eigen::VectorXd scale = Eigen::VectorXd::Ones(m);
    for (int i = 0; i < m; i++) {

        // Variance already explained by a model: leave the channel alone.
        if (i < (int)explained.size() && explained[i]) {
            _channels[i].gain = 1.0;
            _channels[i].r_scale = 1.0;
            continue;
        }

        // Correntropy weight. R <- R / G, per the gain-form equivalence in the
        // header, so a gross residual widens its own covariance instead of
        // dragging the state toward itself.
        double g = std::exp(-(n(i) * n(i)) / (2.0 * sigma * sigma));
        g = std::max(g, _opts.gain_min);
        _channels[i].gain = g;
        scale(i) /= g;

        // Innovation-based noise estimate. mean(n^2) is 1 when the filter is
        // consistent, so the clamped departure from 1 corrects a persistently
        // mis-stated R without letting a short window replace it outright. The
        // clamp floor is 1.0 (see OptionsCorrentropy): widening only.
        if (window_full) {
            const int n_buf = (int)_window.size();
            const int start = std::max(0, n_buf - _opts.recent);
            double sum_sq = 0.0;
            int count = 0;
            for (int k = start; k < n_buf; k++) {
                sum_sq += _window[k](i) * _window[k](i);
                count++;
            }
            double r_scale = 1.0;
            if (count > 0) {
                r_scale = std::min(std::max(sum_sq / count, _opts.r_scale_min),
                                   _opts.r_scale_max);
            }
            _channels[i].r_scale = r_scale;
            scale(i) *= r_scale;
        } else {
            _channels[i].r_scale = 1.0;
        }
    }

    // ---- 3) apply as a congruence: R <- D R D, D = diag(sqrt(scale)) ----
    // R_ij *= sqrt(s_i s_j). This scales the cross-terms consistently with the
    // diagonal and keeps R positive definite for any positive scale, which
    // scaling R(i,i) alone does not.
    if (!scale.isOnes()) {
        const Eigen::VectorXd d = scale.cwiseSqrt();
        R = R.cwiseProduct(d * d.transpose());
    }

    // ---- 4) numerical guard ----
    // StateHelper::EKFUpdate kills the process when the state covariance comes
    // back non-PSD, so never hand it an R that is not positive definite.
    R = 0.5 * (R + R.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(R);
    if (es.info() == Eigen::Success) {
        Eigen::VectorXd ev = es.eigenvalues();
        const double ev_floor = 1e-9 * std::max(1.0, ev.maxCoeff());
        if (ev.minCoeff() < ev_floor) {
            ev = ev.cwiseMax(ev_floor);
            R = es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
        }
    }

    // ---- 5) advance the window ----
    _window.push_back(n);
    while ((int)_window.size() > _opts.window) {
        _window.pop_front();
    }
}
