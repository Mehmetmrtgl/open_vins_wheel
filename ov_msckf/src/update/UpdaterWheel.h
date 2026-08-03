#ifndef OV_MSCKF_UPDATER_WHEEL_H
#define OV_MSCKF_UPDATER_WHEEL_H

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <mutex>
#include <deque>
#include <boost/math/distributions/chi_squared.hpp>

#include "state/State.h"
#include "types/Type.h"
#include "utils/sensor_data.h"

#include "OptionsPlatform.h"
#include "PlatformMotionModel.h"

namespace ov_msckf {


/**
 * @brief Wheel odometry updater for OpenVINS
 * * This class integrates wheel odometry measurements to update the IMU state.
 * It uses preintegration between two clone times and performs EKF update.
 */
class UpdaterWheel {
public:
    /**
     * @brief Constructor
     * @param state Pointer to the state object
     */
    UpdaterWheel(std::shared_ptr<ov_msckf::State> state);

    // CHANGED: Explicitly use ov_core::OdometryData
    /**
     * @brief Feed new odometry measurement
     * @param message Odometry data
     * @param oldest_time Oldest clone time for cleaning old data
     */
    void feed_measurement(const ov_core::OdometryData& message, double oldest_time);

    /**
     * @brief Try to update the state with available measurements
     */
    void try_update();

    /// Whether the system is currently in pure-rotation mode
    bool is_pure_rotation() const { return last_was_pure_rotation; }

    /**
     * @brief Set extrinsic calibration (IMU to Odometry frame)
     * @param T_imu_odom 4x4 transformation matrix
     */
    void set_extrinsics(const Eigen::Matrix4d& T) {
        T_imu_odom = T;
    }

    /**
     * @brief Set noise parameters
     * @param gyro_noise Gyroscope noise (rad/s)
     * @param vel_noise Velocity noise (m/s)
     */
    void set_noise(double gyro_noise, double vel_noise, double pos_noise) {
        noise_gyro = gyro_noise;
        noise_vel  = vel_noise;
        noise_pos  = pos_noise;
        PRINT_INFO("[WHEEL] Noise SET: w=%.4f (ang.z)  v=%.4f (lin.x)  p=%.4f (near-zero axes)\n",
                   noise_gyro, noise_vel, noise_pos);
    }

    void set_chi2_mult(double mult) { chi2_mult = mult; }

    void set_turn_detection(bool enable, double threshold) {
        turn_detection_enabled = enable;
        turn_ang_threshold     = threshold;
        PRINT_INFO("[WHEEL] Turn detection: %s (threshold=%.3f rad/s)\n",
                   enable ? "ON" : "OFF", threshold);
    }

    /// Attach the platform motion model (call from VioManager after construction).
    /// nullptr keeps the legacy path: isotropic-per-role Q from noise_gyro /
    /// noise_vel / noise_pos and a fixed R at the EKF update.
    void set_platform(std::shared_ptr<PlatformMotionModel> model) { platform = model; }

private:
    /// Platform motion model; nullptr = legacy behaviour
    std::shared_ptr<PlatformMotionModel> platform;

    /**
     * @brief Update state between two clone times
     * @param time0 Start time
     * @param time1 End time
     * @return Success status
     */
    bool update(double time0, double time1);

    // CHANGED: Explicitly use ov_core::OdometryData
    /**
     * @brief Select odometry data between two times
     * @param time0 Start time
     * @param time1 End time
     * @param data_vec Output vector of selected data
     * @return Success status
     */
    bool select_odometry_data(double time0, double time1, std::vector<ov_core::OdometryData>& data_vec);

    /**
     * @brief Clean old measurements
     * @param oldest_time Threshold time
     */
    void clean_old_measurements(double oldest_time);

    void preintegration_RK4(double dt, const ov_core::OdometryData& data1, const ov_core::OdometryData& data2);

    // CHANGED: Explicitly use ov_core::OdometryData
    /**
     * @brief Preintegrate odometry measurements (simple version)
     * @param dt Time step
     * @param data1 Start measurement
     * @param data2 End measurement
     */
    void preintegration_3D(double dt, const ov_core::OdometryData& data1, const ov_core::OdometryData& data2);

    /**
     * @brief Compute linear system for EKF update
     * @param H Measurement Jacobian matrix (output)
     * @param res Residual vector (output)
     * @param x_order State variables to update (output)
     * @param time0 Start time
     * @param time1 End time
     * @return Success status
     */
    bool compute_linear_system(Eigen::MatrixXd& H, Eigen::VectorXd& res,
                                std::vector<std::shared_ptr<ov_type::Type>>& x_order,
                                double time0, double time1);

    // CHANGED: Fixed return type to ov_core::OdometryData
    /**
     * @brief Interpolate odometry data
     * @param data1 First data point
     * @param data2 Second data point
     * @param timestamp Target timestamp
     * @return Interpolated data
     */
    ov_core::OdometryData interpolate_data(const ov_core::OdometryData& data1, const ov_core::OdometryData& data2, double timestamp);

    // ========== Utility functions ==========

    /**
     * @brief Exponential map for SO(3)
     */
    Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega);

    /**
     * @brief Logarithm map for SO(3)
     */
    Eigen::Vector3d log_so3(const Eigen::Matrix3d& R);

    /**
     * @brief Skew-symmetric matrix
     */
    Eigen::Matrix3d skew_x(const Eigen::Vector3d& v);

    /**
     * @brief Right Jacobian of SO(3)
     */
    Eigen::Matrix3d Jr_so3(const Eigen::Vector3d& w);

    /**
     * @brief Inverse of Right Jacobian of SO(3)
     */
    Eigen::Matrix3d Jr_so3_inv(const Eigen::Vector3d& w);

    /**
     * @brief Rotation matrix to quaternion (Hamilton convention)
     */
    Eigen::Vector4d rot_2_quat(const Eigen::Matrix3d& R);

    /**
     * @brief Quaternion to rotation matrix
     */
    Eigen::Matrix3d quat_2_Rot(const Eigen::Vector4d& q);

    /**
     * @brief Quaternion multiplication
     */
    Eigen::Vector4d quat_multiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);

    /**
     * @brief Normalize quaternion
     */
    Eigen::Vector4d quatnorm(const Eigen::Vector4d& q);

    /**
     * @brief Omega matrix for quaternion derivative
     */
    Eigen::Matrix4d Omega(const Eigen::Vector3d& w);

    // ========== Member variables ==========

    /// Pointer to state
    std::shared_ptr<ov_msckf::State> state;

    /// Odometry data buffer
    std::mutex odometry_data_mtx;
    std::vector<ov_core::OdometryData> odometry_data;

    /// Last updated clone time
    double last_updated_clone_time;

    /// Preintegrated values
    Eigen::Vector3d delta_p;
    Eigen::Matrix3d delta_R;
    Eigen::Matrix<double, 6, 6> covariance;

    /// Extrinsic calibration (IMU to Odometry)
    Eigen::Matrix4d T_imu_odom = Eigen::Matrix4d::Identity();

    /// Noise parameters (ackermann-specific)
    double noise_gyro = 0.2;  ///< angular.z (yaw rate from wheels)
    double noise_vel  = 0.5;  ///< linear.x  (forward velocity from encoder)
    double noise_pos  = 0.1;  ///< near-zero axes: angular.x/y, linear.y/z

    /// Chi2 multiplier for outlier rejection (same as MINS/UpdaterMSCKF)
    double chi2_mult = 15.0;

    /// Chi2 95% lookup table, indexed by DOF (precomputed in constructor via boost)
    std::map<int, double> chi_squared_table;

    /// Turn detection (config-driven)
    bool   turn_detection_enabled = false;
    double turn_ang_threshold     = 0.3;   ///< rad/s — from wheel_config.yaml
    bool   last_was_pure_rotation = false;

    /// Counts successful EKF updates (diagnostic only)
    int update_count = 0;

};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_WHEEL_H
