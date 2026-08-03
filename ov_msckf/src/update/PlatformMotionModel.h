#ifndef OV_MSCKF_PLATFORM_MOTION_MODEL_H
#define OV_MSCKF_PLATFORM_MOTION_MODEL_H

#include <Eigen/Dense>
#include <deque>
#include <vector>

#include "OptionsPlatform.h"

namespace ov_msckf {

/**
 * @brief Platform-aware adaptive noise model for the 6-dof wheel/leg
 *        odometry residual [rot(3), pos(3)].
 *
 * Extends the openVINS-adaptive branch's correntropy filter in three ways:
 *
 *  1) PER-AXIS instead of scalar: correntropy weights, innovation windows and
 *     R scales are maintained independently for each of the 6 residual axes.
 *     A scalar norm mixes axes, so a legged robot's real vertical oscillation
 *     would drag down the weight of perfectly good forward-motion axes.
 *
 *  2) PERIODICITY DETECTION: for each axis we keep a ring buffer of raw
 *     innovations and compute a normalized autocorrelation. If a strong peak
 *     exists at a lag whose frequency lies inside the configured gait band,
 *     the innovation on that axis is STRUCTURED MOTION (gait), not noise.
 *     Correntropy down-weighting is then suppressed on that axis and the
 *     oscillation power is instead absorbed into the expected measurement
 *     variance, so chi2 gating and the EKF gain stay consistent.
 *
 *  3) PLATFORM PRIOR: constrained axes (e.g. car lateral/vertical velocity)
 *     get their base R inflated from OptionsPlatform::noise_v_axis, encoding
 *     "signal on this channel is noise" without discarding the message.
 */
class PlatformMotionModel {

public:
    explicit PlatformMotionModel(const OptionsPlatform &opts) : _opts(opts) {}

    /**
     * @brief Per-axis gait/periodicity state for diagnostics.
     */
    struct AxisState {
        bool   periodic = false;   ///< gait-band periodicity detected
        double freq_hz = 0.0;      ///< detected dominant frequency
        double osc_var = 0.0;      ///< estimated oscillation variance
        double corr_gain = 1.0;    ///< last correntropy weight G in (0,1]
        double r_scale = 1.0;      ///< last sliding-window R scale
    };

    /**
     * @brief Build the base 6x6 measurement covariance from the platform
     *        profile and the integration step, replacing the isotropic
     *        Q of the original UpdaterWheel::preintegration_RK4.
     * @param dt   Integration step
     * @return     6x6 continuous-time noise covariance (rot block, pos block)
     */
    Eigen::Matrix<double, 6, 6> base_noise(double dt) const;

    /**
     * @brief Feed one raw 6-dof residual (post-update innovation proxy) and
     *        its timestamp so the periodicity buffers advance.
     */
    void push_innovation(double timestamp, const Eigen::Matrix<double, 6, 1> &innov);

    /**
     * @brief Platform-aware adaptation of the measurement covariance.
     *
     * Mirrors the adaptive branch flow (weight -> R scaling -> sliding-window
     * R estimate) but per axis and gait-aware:
     *
     *   for each axis i:
     *     if axis is gait-periodic:  R_ii += osc_var_i          (model it)
     *     else:                      G_i = exp(-n_i^2 / 2 s_i^2)
     *                                R_ii *= 1 / max(G_i, eps)^2  (down-weight)
     *     R_ii *= clamp(recent mean normalized innov^2, min, max)
     *
     * @param R      6x6 covariance to adapt in place
     * @param res    Current 6-dof residual (pre-update)
     */
    void adapt_R(Eigen::Matrix<double, 6, 6> &R,
                 const Eigen::Matrix<double, 6, 1> &res);

    /**
     * @brief Correntropy weighting for the nonholonomic constraint's own R.
     *
     * Same mechanism as the noise path of adapt_R, but for the constraint
     * pseudo-measurement rather than the odometry residual, and with its own
     * state so wheel and constraint innovations never mix. The constraint
     * residual is sharply peaked at zero and violated in bursts, so a fixed
     * sigma cannot describe it; scaling per sample lets sigma be chosen for
     * the bulk while turns, bumps and camber soften their own weight.
     *
     * R is diagonal by construction in UpdaterPlatform, so scaling R(k,k)
     * alone is exact here — no congruence transform is needed.
     *
     * @param R     m x m constraint covariance, adapted in place
     * @param res   m-vector constraint residual
     * @param axes  Body axis index for each row (for per-axis diagnostics)
     */
    void adapt_constraint_R(Eigen::MatrixXd &R, const Eigen::VectorXd &res,
                            const std::vector<int> &axes);

    /// Diagnostics access
    const AxisState &axis_state(int i) const { return _axes[i]; }

    /// Last correntropy weight applied to the constraint on the given body axis
    double constraint_gain(int axis) const { return _constraint_gain[axis]; }

    /// True once the innovation window has filled (adaptation active)
    bool active() const { return (int)_innov_window.size() >= _opts.corr_window; }

private:
    /**
     * @brief Normalized autocorrelation periodicity test on one axis buffer.
     * @param buf        Sample buffer (oldest first)
     * @param mean_dt    Mean sample spacing (s)
     * @param freq_out   Detected frequency (Hz) if periodic
     * @param power_out  Variance of the periodic component
     * @return true if a gait-band periodic component was found
     */
    bool detect_periodicity(const std::deque<double> &buf, double mean_dt,
                            double &freq_out, double &power_out) const;

    const OptionsPlatform &_opts;

    /// Sliding window of normalized innovation vectors (adaptive-branch style)
    std::deque<Eigen::Matrix<double, 6, 1>> _innov_window;

    /// Per-axis raw innovation ring buffers + timestamps for periodicity
    std::deque<double> _raw_buf[6];
    std::deque<double> _time_buf;

    AxisState _axes[6];

    /// Last constraint correntropy weight per body axis (diagnostics)
    double _constraint_gain[3] = {1.0, 1.0, 1.0};
};

} // namespace ov_msckf

#endif // OV_MSCKF_PLATFORM_MOTION_MODEL_H
