#ifndef OV_MSCKF_UPDATER_PLATFORM_H
#define OV_MSCKF_UPDATER_PLATFORM_H

#include <Eigen/Dense>
#include <fstream>
#include <memory>

#include "OptionsPlatform.h"
#include "PlatformMotionModel.h"

namespace ov_msckf {

class State;

/**
 * @brief Nonholonomic platform-constraint updater.
 *
 * Turns "this robot cannot move along axis k" into an EKF pseudo-measurement:
 *
 *      z_k = e_k^T * v_O_in_O = 0,     n_k ~ N(0, sigma_k^2)
 *
 * where v_O_in_O is the velocity of the odometry-frame origin expressed in
 * the odometry frame:
 *
 *      v_O_in_O = R_ItoO * ( R_GtoI * v_I_in_G + skew(w_hat) * p_O_in_I )
 *
 * with w_hat = w_m - b_g (bias-corrected gyro). For a car the constrained
 * axes are y (lateral) and z (vertical); for a differential base only y.
 * This mirrors how UpdaterZeroVelocity exploits v = 0, but restricted to a
 * subset of body axes and active during normal driving, so the information
 * "lateral motion is impossible" continuously corrects velocity drift.
 *
 * The update touches [theta_GtoI, v_I_in_G, b_g] of the active IMU state.
 */
class UpdaterPlatform {

public:
    UpdaterPlatform(std::shared_ptr<State> state, const OptionsPlatform &opts)
        : _state(state), _opts(opts) {
        if (!_opts.constraint_log_path.empty()) {
            _log.open(_opts.constraint_log_path, std::ios::out | std::ios::trunc);
            if (_log.is_open())
                _log << "t,axis0,res0,axis1,res1,vx_pred,whx,why,whz,chi2,applied" << std::endl;
        }
    }

    /// Extrinsics, read with exactly the same convention as UpdaterWheel:
    ///   T = [R_OtoI | p_OinI ; 0 0 0 1]   ("from odometry/platform to IMU")
    ///   R_OtoI : rotation from the platform frame to the IMU frame
    ///   p_OinI : platform-frame origin expressed in the IMU frame
    /// Both updaters must agree here, otherwise T_imu_platform and T_imu_wheel
    /// would silently mean different things for the same 4x4 block of yaml.
    void set_extrinsics(const Eigen::Matrix4d &T_imu_odom) {
        Eigen::Matrix3d R_OtoI = T_imu_odom.block<3, 3>(0, 0);
        _R_ItoO = R_OtoI.transpose();
        _p_OinI = T_imu_odom.block<3, 1>(0, 3);
    }

    /**
     * @brief Try a constraint update at the given time.
     * @param w_m       Latest gyro measurement (IMU frame, rad/s), needed for
     *                  the lever-arm term. Pass the closest raw IMU sample.
     * @return true if an update was applied
     */
    bool try_update(const Eigen::Vector3d &w_m);

    /// Attach the platform model so the constraint can use its adaptive
    /// weighting (optional; without it a fixed constraint_sigma is used).
    void set_model(std::shared_ptr<PlatformMotionModel> model) { _model = model; }

private:
    std::shared_ptr<State> _state;
    const OptionsPlatform &_opts;
    std::shared_ptr<PlatformMotionModel> _model;   // nullptr = fixed sigma

    Eigen::Matrix3d _R_ItoO = Eigen::Matrix3d::Identity();
    Eigen::Vector3d _p_OinI = Eigen::Vector3d::Zero();

    /// 95% chi2 gate for 1 and 2 dof
    const double _chi2_1dof = 3.841;
    const double _chi2_2dof = 5.991;

    /// How many numerical Jacobian checks have been printed
    int _jac_checks_done = 0;

    /// State time of the last applied constraint update (for constraint_min_dt)
    double _last_update_time = -1.0;

    /// Per-evaluation residual log (open iff constraint_log_path is non-empty).
    /// One row per constraint evaluation, accepted or rejected, consumed
    /// offline by ov_msckf/scripts/residual_diagnostics.py.
    std::ofstream _log;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_PLATFORM_H
