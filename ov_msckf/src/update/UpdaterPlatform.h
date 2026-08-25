#ifndef OV_MSCKF_UPDATER_PLATFORM_H
#define OV_MSCKF_UPDATER_PLATFORM_H

#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <memory>

#include "OptionsPlatform.h"
#include "PlatformMotionModel.h"

namespace ov_msckf {

class State;

/**
 * @brief Correntropy-weighted Kalman update driven by the platform's kinematics.
 *
 * The morphology supplies a measurement no sensor has to provide: on an axis the
 * platform cannot travel along, the body velocity is zero.
 *
 *      z_a = e_a^T * v_O_in_O = 0,     n_a ~ N(0, sigma_a^2)
 *
 * where v_O_in_O is the velocity of the platform-frame origin expressed in that
 * frame,
 *
 *      v_O_in_O = R_ItoO * ( R_GtoI * v_I_in_G + skew(w_hat) * p_O_in_I )
 *
 * with w_hat = w_m - b_g (bias-corrected gyro). The update touches
 * [theta_GtoI, v_I_in_G, b_g] of the active IMU state, so "lateral motion is
 * impossible" continuously corrects velocity drift the way UpdaterZeroVelocity
 * exploits v = 0, but on a subset of body axes and during normal driving.
 *
 * The measurement is not Gaussian: on smooth straight motion the residual sits
 * essentially at zero, and it is violated in bursts (turns, bumps, camber, a
 * skid). One fixed sigma cannot describe both — tight enough for the bulk makes
 * every burst an outlier, loose enough for the bursts makes the bulk contribute
 * nothing. That is exactly the regime the correntropy criterion exists for, so
 * the weighting is not an option layered on the update, it IS the update:
 * PlatformMotionModel::adapt_R fixes the covariance this step is performed with,
 * and only violations too large for the weighting to absorb reach the chi2 gate.
 *
 * Independent of wheel odometry in both directions: it needs no odometry stream
 * to run, and it does not reach into the wheel updater when one is present.
 */
class UpdaterPlatform {

public:
    UpdaterPlatform(std::shared_ptr<State> state, const OptionsPlatform &opts);

    /**
     * @brief Evaluate and, if accepted, apply the platform update.
     * @param w_m  Latest gyro measurement (IMU frame, rad/s), needed for the
     *             lever-arm term. Pass the closest raw IMU sample.
     * @return true if an update was applied
     */
    bool try_update(const Eigen::Vector3d &w_m);

    /// True when the morphology pins at least one axis, i.e. there is anything
    /// for this updater to do at all.
    bool has_measurement() const { return !_model->axes().empty(); }

    /// The morphology + correntropy model backing this updater (diagnostics)
    std::shared_ptr<PlatformMotionModel> model() const { return _model; }

private:
    std::shared_ptr<State> _state;
    OptionsPlatform _opts;
    std::shared_ptr<PlatformMotionModel> _model;

    /// Extrinsics, read with exactly the same convention as UpdaterWheel:
    ///   T = [R_OtoI | p_OinI ; 0 0 0 1]   ("from platform to IMU")
    ///   R_OtoI : rotation from the platform frame to the IMU frame
    ///   p_OinI : platform-frame origin expressed in the IMU frame
    Eigen::Matrix3d _R_ItoO = Eigen::Matrix3d::Identity();
    Eigen::Vector3d _p_OinI = Eigen::Vector3d::Zero();

    /// 95% chi2 quantiles by dof, precomputed for the 1..3 axes possible here
    std::map<int, double> _chi2_table;

    /// How many numerical Jacobian checks have been printed
    int _jac_checks_done = 0;

    /// State time of the last applied update (for update_min_dt)
    double _last_update_time = -1.0;

    /// Per-evaluation residual log (open iff log_path is non-empty). One row per
    /// evaluation, accepted or rejected, consumed offline by
    /// ov_msckf/scripts/residual_diagnostics.py.
    std::ofstream _log;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_PLATFORM_H
