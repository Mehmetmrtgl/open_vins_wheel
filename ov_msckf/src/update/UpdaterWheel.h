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

namespace ov_msckf {


/**
 * @brief Wheel odometry updater for OpenVINS
 *
 * This class integrates wheel odometry measurements to update the IMU state.
 * It uses preintegration between two clone times and performs EKF update.
 *
 * Self-contained: it owns its own noise model and knows nothing about the
 * platform motion model. The two are separate integrations of separate
 * information — an odometry stream, and the vehicle's kinematics — and either
 * runs with or without the other.
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
    /**
     * @brief Set the per-axis body-frame measurement noise for the odometry
     *        channel. This is where the vehicle's kinematics enter the wheel
     *        update: the axes the odometry reports as zero carry the
     *        constraint, so their sigma says how far the true body velocity may
     *        depart from it, not how noisy a dead channel is.
     * @param w_axis Angular sigmas [x, y, z] (rad/s)
     * @param v_axis Linear sigmas  [x, y, z] (m/s)
     */
    void set_noise_axis(const Eigen::Vector3d &w_axis, const Eigen::Vector3d &v_axis) {
        noise_w_axis = w_axis;
        noise_v_axis = v_axis;
        PRINT_INFO("[WHEEL] Noise SET: w=[%.4f %.4f %.4f]  v=[%.4f %.4f %.4f]\n",
                   noise_w_axis(0), noise_w_axis(1), noise_w_axis(2),
                   noise_v_axis(0), noise_v_axis(1), noise_v_axis(2));
    }

    void set_chi2_mult(double mult) { chi2_mult = mult; }

    void set_turn_detection(bool enable, double threshold) {
        turn_detection_enabled = enable;
        turn_ang_threshold     = threshold;
        PRINT_INFO("[WHEEL] Turn detection: %s (threshold=%.3f rad/s)\n",
                   enable ? "ON" : "OFF", threshold);
    }

private:
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

    /// Per-axis body-frame measurement noise for the odometry channel
    Eigen::Vector3d noise_w_axis = Eigen::Vector3d(0.1, 0.1, 0.2);  ///< angular x, y, z
    Eigen::Vector3d noise_v_axis = Eigen::Vector3d(0.5, 0.1, 0.1);  ///< linear x, y, z

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
