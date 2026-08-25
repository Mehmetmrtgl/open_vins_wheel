#ifndef OV_MSCKF_PLATFORM_MOTION_MODEL_H
#define OV_MSCKF_PLATFORM_MOTION_MODEL_H

#include <Eigen/Dense>
#include <deque>
#include <vector>

#include "CorrentropyFilter.h"
#include "OptionsPlatform.h"

namespace ov_msckf {

/**
 * @brief Robot morphology as a measurement model, with correntropy-weighted
 *        adaptive noise.
 *
 * Owns three things and nothing else:
 *
 *  1) WHICH AXES the platform kinematics pin, and how tightly. A car cannot
 *     slide sideways, so v_y = 0 is a genuine measurement available at every
 *     instant with no sensor attached to it; meas_sigma states how far reality
 *     may depart from that (tyre slip, camber, calibration residue).
 *
 *  2) GAIT PERIODICITY. Per axis, a ring buffer of raw residuals and a
 *     normalized autocorrelation over the configured gait band. If a strong
 *     peak sits at a lag in that band, the residual on that axis is STRUCTURED
 *     MOTION, not noise: its power is added to the expected variance so the
 *     chi2 gate and the gain stay consistent, and the correntropy weighting is
 *     suppressed there so the filter models the gait instead of fighting it.
 *     A wheeled platform has no such component and the test never fires, so
 *     this costs nothing and needs no switch.
 *
 *  3) A CorrentropyFilter over the resulting residual, which is where the
 *     Kalman update and the correntropy weighting meet — see that class for the
 *     gain-form equivalence.
 *
 * It holds no wheel odometry state and no reference to the wheel updater: the
 * kinematic facts above are true of the vehicle, not of any sensor on it.
 */
class PlatformMotionModel {

public:
    explicit PlatformMotionModel(const OptionsPlatform &opts);

    /// Per-axis gait state, for diagnostics and for the update's own use
    struct AxisState {
        bool periodic = false;   ///< gait-band periodicity detected
        double freq_hz = 0.0;    ///< detected dominant frequency
        double osc_var = 0.0;    ///< estimated oscillation variance
        double gain = 1.0;       ///< last correntropy weight G in (0,1]
        double r_scale = 1.0;    ///< last innovation-based noise scale
    };

    /// Body axes this morphology constrains, ascending. Empty for a platform
    /// with no kinematic knowledge, in which case there is nothing to update.
    const std::vector<int> &axes() const { return _axes_idx; }

    /// Diagonal measurement covariance for those axes: meas_sigma, widened by
    /// bias_sigma because the residual handed to the filter has been corrected
    /// by an estimate and is no more certain than that estimate.
    Eigen::MatrixXd base_R() const;

    /**
     * @brief Remove the slowly varying offset from a raw residual.
     *
     * The morphology asserts zero; the sensors report zero plus a drift the
     * model does not resolve (calibration residue, camber, suspension trim, a
     * platform origin offset from the true zero-velocity point). Subtracting the
     * running estimate turns the assertion into "this axis does not depart from
     * its own slow trend", which is the part the data supports — see
     * OptionsPlatform for the urban39 measurement that motivates it.
     *
     * The estimate is built only from residuals already pushed, so this is
     * causal: the correction applied at step k never sees step k's own residual.
     *
     * @param res_raw  m-vector raw residual in axes() order
     * @return         the residual to actually update with
     */
    Eigen::VectorXd debias(const Eigen::VectorXd &res_raw) const;

    /// Current offset estimate on a body axis (diagnostics)
    double bias(int body_axis) const { return _bias[body_axis]; }

    /**
     * @brief Gait absorption followed by correntropy weighting and the
     *        innovation-based noise estimate, applied to R in place.
     *
     *   for each constrained axis a:
     *     if a is gait-periodic:  R += osc_var_a   and skip the weighting
     *     else:                   R <- R / G_a, then times the clamped
     *                             innovation-based scale
     *
     * @param R     m x m covariance in axes() order, adapted in place
     * @param res   m-vector residual in axes() order
     * @param HPHt  m x m state-uncertainty part of the residual covariance, so
     *              the weighting normalizes against S = H P H^T + R
     */
    void adapt_R(Eigen::MatrixXd &R, const Eigen::VectorXd &res,
                 const Eigen::MatrixXd &HPHt);

    /// Feed one evaluated RAW residual (in axes() order) so the periodicity
    /// buffers and the offset estimate advance. Called for rejected updates too:
    /// the gait signature and the drift are properties of the motion, not of
    /// which samples the gate happened to keep.
    ///
    /// The lag-to-frequency mapping assumes these arrive at a steady rate, which
    /// holds for a legged platform (update_min_dt is 0 there, so every frame is
    /// evaluated). A rate-limited wheeled platform that also hits a rejection
    /// streak feeds the buffer at two different rates and the implied frequency
    /// would be wrong — but no gait exists there to detect in the first place.
    void push_residual(double timestamp, const Eigen::VectorXd &res);

    /// Diagnostics for a body axis (0=x, 1=y, 2=z)
    const AxisState &axis_state(int body_axis) const { return _axis[body_axis]; }

    /// True once the innovation window has filled (noise estimation active)
    bool active() const { return _corr.active(); }

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

    OptionsPlatform _opts;

    /// Body axis indices selected by meas_mask
    std::vector<int> _axes_idx;

    /// Correntropy weighting + innovation-based noise estimation
    CorrentropyFilter _corr;

    /// Per body axis raw residual ring buffers + shared timestamps
    std::deque<double> _raw_buf[3];
    std::deque<double> _time_buf;

    /// Slowly varying per-axis offset estimate, and the time it last advanced
    double _bias[3] = {0.0, 0.0, 0.0};
    double _bias_time = -1.0;

    AxisState _axis[3];
};

} // namespace ov_msckf

#endif // OV_MSCKF_PLATFORM_MOTION_MODEL_H
