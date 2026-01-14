/**

#include "UpdaterWheel.h"
#include "utils/print.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace Eigen;


UpdaterWheel::UpdaterWheel(std::shared_ptr<State> state) : state(state) {
    // Initialize with identity/zero
    last_updated_clone_time = -1.0;
}


void UpdaterWheel::feed_measurement(const OdometryData& message, double oldest_time) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);
    odometry_data.push_back(message);

    clean_old_measurements(oldest_time);
}


void UpdaterWheel::try_update() {
    // Check if we have valid clone times
    if (state->_clones_IMU.empty()) {
        return;
    }

    // Get the oldest and newest clone times
    double oldest_time = state->_clones_IMU.begin()->first;
    double newest_time = state->_clones_IMU.rbegin()->first;

    // Check last updated clone time still exists in the state
    if (last_updated_clone_time < 0 ||
        state->_clones_IMU.find(last_updated_clone_time) == state->_clones_IMU.end()) {
        // Initialize to oldest time if not set or if clone was removed
        last_updated_clone_time = oldest_time;
        return;
    }

    // Iterate through all clones and update sequentially
    for (auto it = state->_clones_IMU.begin(); it != state->_clones_IMU.end(); ++it) {
        // Skip if we already updated this clone
        if (it->first <= last_updated_clone_time) {
            continue;
        }

        // Try to update between last_updated_clone_time and current clone
        if (!update(last_updated_clone_time, it->first)) {
            break; // Stop if update fails
        }
    }
}


bool UpdaterWheel::update(double time0, double time1) {
    // Collect odometry measurements between time0 and time1
    std::vector<OdometryData> data_vec;
    if (!select_odometry_data(time0, time1, data_vec) || data_vec.size() < 2) {
        PRINT_DEBUG("[WHEEL]: Not enough odometry measurements between %.3f and %.3f\n", time0, time1);
        return false;
    }

    // Reset preintegration values
    delta_p.setZero();
    delta_R.setIdentity();
    covariance.setZero();

    // Preintegrate all measurements
    for (size_t i = 0; i + 1 < data_vec.size(); ++i) {
        const auto& d1 = data_vec[i];
        const auto& d2 = data_vec[i + 1];

        double dt = d2.timestamp - d1.timestamp;
        if (dt <= 1e-6) continue; // Skip very small time steps

        preintegration_3D(dt, d1, d2);
    }

    // Compute linear system (H matrix and residuals)
    Eigen::MatrixXd H;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<ov_type::Type>> x_order;

    if (!compute_linear_system(H, res, x_order, time0, time1)) {
        PRINT_DEBUG("[WHEEL]: Failed to compute linear system\n");
        return false;
    }

    // Perform Chi-squared check and EKF update
    // For now, we'll skip chi-squared check and directly update
    // You can add chi-squared check later if needed
    StateHelper::EKFUpdate(state, x_order, H, res, covariance);

    // Record last updated time
    last_updated_clone_time = time1;
    return true;
}


bool UpdaterWheel::select_odometry_data(double time0, double time1,
                                         std::vector<OdometryData>& data_vec) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);

    if (odometry_data.empty() || time1 <= time0) {
        return false;
    }

    // Check if we have data covering the time range
    if (odometry_data.back().timestamp < time1 || odometry_data.front().timestamp > time0) {
        return false;
    }

    // Find measurements within the time range
    // We need to interpolate at boundaries if necessary
    bool found_start = false;
    bool found_end = false;

    for (size_t i = 0; i < odometry_data.size(); ++i) {
        const auto& curr = odometry_data[i];

        // Handle start boundary
        if (!found_start && i + 1 < odometry_data.size()) {
            const auto& next = odometry_data[i + 1];
            if (curr.timestamp <= time0 && next.timestamp > time0) {
                // Interpolate at time0
                OdometryData interp = interpolate_data(curr, next, time0);
                data_vec.push_back(interp);
                found_start = true;
                continue;
            }
        }

        // Add measurements in the middle
        if (curr.timestamp >= time0 && curr.timestamp <= time1) {
            data_vec.push_back(curr);
        }

        // Handle end boundary
        if (curr.timestamp < time1 && i + 1 < odometry_data.size()) {
            const auto& next = odometry_data[i + 1];
            if (next.timestamp >= time1 && curr.timestamp < time1) {
                // Interpolate at time1
                OdometryData interp = interpolate_data(curr, next, time1);
                data_vec.push_back(interp);
                found_end = true;
                break;
            }
        }
    }

    // Ensure we have at least 2 measurements
    return data_vec.size() >= 2;
}


void UpdaterWheel::clean_old_measurements(double oldest_time) {
    if (oldest_time < 0) return;

    auto it = odometry_data.begin();
    while (it != odometry_data.end()) {
        // Keep a buffer of older measurements for interpolation
        if (it->timestamp < oldest_time - 1.0) {
            it = odometry_data.erase(it);
        } else {
            ++it;
        }
    }
}


void UpdaterWheel::preintegration_3D(double dt, const OdometryData& data1, const OdometryData& data2) {
    // Get angular and linear velocities from odometry
    // Average the velocities for better integration (trapezoidal rule)
    Eigen::Vector3d w1 = data1.angular_velocity;
    Eigen::Vector3d v1 = data1.linear_velocity;
    Eigen::Vector3d w2 = data2.angular_velocity;
    Eigen::Vector3d v2 = data2.linear_velocity;

    // Average velocities
    Eigen::Vector3d w_avg = 0.5 * (w1 + w2);
    Eigen::Vector3d v_avg = 0.5 * (v1 + v2);

    // Transform velocities from odometry frame to IMU frame using fixed extrinsics
    Eigen::Matrix3d R_ItoO = T_imu_odom.block<3,3>(0,0);
    Eigen::Vector3d p_IinO = T_imu_odom.block<3,1>(0,3);

    Eigen::Vector3d w_imu = R_ItoO.transpose() * w_avg;
    Eigen::Vector3d v_imu = R_ItoO.transpose() * (v_avg - w_avg.cross(p_IinO));

    // Preintegrate rotation using exponential map
    Eigen::Matrix3d dR = exp_so3(w_imu * dt);
    Eigen::Matrix3d R_new = delta_R * dR;

    // Preintegrate position
    Eigen::Vector3d p_new = delta_p + delta_R * v_imu * dt;

    // Compute Jacobians for covariance propagation
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3,3>(0,0) = dR.transpose();
    F.block<3,3>(3,0) = -delta_R * skew_x(v_imu * dt);

    // Noise matrix
    Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Zero();
    G.block<3,3>(0,0) = delta_R * dt;
    G.block<3,3>(3,3) = delta_R * dt;

    // Process noise covariance (use fixed noise parameters)
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.block<3,3>(0,0) = (noise_gyro * noise_gyro / dt) * Eigen::Matrix3d::Identity();
    Q.block<3,3>(3,3) = (noise_vel * noise_vel / dt) * Eigen::Matrix3d::Identity();

    // Propagate covariance
    covariance = F * covariance * F.transpose() + G * Q * G.transpose();

    // Ensure symmetry
    covariance = 0.5 * (covariance + covariance.transpose());

    // Update preintegrated values
    delta_R = R_new;
    delta_p = p_new;
}


bool UpdaterWheel::compute_linear_system(Eigen::MatrixXd& H, Eigen::VectorXd& res,
                                          std::vector<std::shared_ptr<ov_type::Type>>& x_order,
                                          double time0, double time1) {
    // Check if clones exist
    if (state->_clones_IMU.find(time0) == state->_clones_IMU.end() ||
        state->_clones_IMU.find(time1) == state->_clones_IMU.end()) {
        return false;
    }

    // Get clone poses
    auto clone0 = state->_clones_IMU.at(time0);
    auto clone1 = state->_clones_IMU.at(time1);

    // Extract poses (using FEJ values for linearization)
    Eigen::Matrix3d R_GtoI0 = clone0->Rot();
    Eigen::Vector3d p_I0inG = clone0->pos();
    Eigen::Matrix3d R_GtoI1 = clone1->Rot();
    Eigen::Vector3d p_I1inG = clone1->pos();

    // Transform to odometry frame
    Eigen::Matrix3d R_ItoO = T_imu_odom.block<3,3>(0,0);
    Eigen::Vector3d p_IinO = T_imu_odom.block<3,1>(0,3);
    Eigen::Vector3d p_OinI = -R_ItoO.transpose() * p_IinO;

    // Compute expected relative transformation
    Eigen::Matrix3d R_O0toO1 = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    Eigen::Vector3d p_O1inO0 = R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * p_OinI
                                                     - p_I0inG - R_GtoI0.transpose() * p_OinI);

    // Compute residuals (measurement - estimate)
    res = Eigen::VectorXd::Zero(6);
    res.segment<3>(0) = -log_so3(delta_R * R_O0toO1.transpose()); // Rotation residual
    res.segment<3>(3) = delta_p - p_O1inO0;                        // Position residual

    // Compute Jacobians
    int H_size = 12; // 6 for each pose (rotation + position)
    H = Eigen::MatrixXd::Zero(6, H_size);

    // Jacobian wrt pose0
    Eigen::Matrix3d J_rot_R0 = -R_ItoO * R_GtoI1 * R_GtoI0.transpose();
    Eigen::Matrix3d J_pos_R0 = R_ItoO * skew_x(R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * p_OinI - p_I0inG));
    Eigen::Matrix3d J_pos_p0 = -R_ItoO * R_GtoI0;

    H.block<3,3>(0,0) = J_rot_R0;
    H.block<3,3>(3,0) = J_pos_R0;
    H.block<3,3>(3,3) = J_pos_p0;

    // Jacobian wrt pose1
    Eigen::Matrix3d J_rot_R1 = R_ItoO;
    Eigen::Matrix3d J_pos_R1 = -R_ItoO * R_GtoI0 * R_GtoI1.transpose() * skew_x(p_OinI);
    Eigen::Matrix3d J_pos_p1 = R_ItoO * R_GtoI0;

    H.block<3,3>(0,6) = J_rot_R1;
    H.block<3,3>(3,6) = J_pos_R1;
    H.block<3,3>(3,9) = J_pos_p1;

    // Set state ordering
    x_order.clear();
    x_order.push_back(clone0);
    x_order.push_back(clone1);

    return true;
}


OdometryData UpdaterWheel::interpolate_data(const OdometryData& data1,
                                             const OdometryData& data2,
                                             double timestamp) {
    // Linear interpolation factor
    double lambda = (timestamp - data1.timestamp) / (data2.timestamp - data1.timestamp);
    lambda = std::max(0.0, std::min(1.0, lambda)); // Clamp to [0, 1]

    OdometryData interp;
    interp.timestamp = timestamp;
    interp.linear_velocity = (1.0 - lambda) * data1.linear_velocity + lambda * data2.linear_velocity;
    interp.angular_velocity = (1.0 - lambda) * data1.angular_velocity + lambda * data2.angular_velocity;

    return interp;
}


Eigen::Matrix3d UpdaterWheel::exp_so3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();

    if (theta < 1e-6) {
        return Eigen::Matrix3d::Identity() + skew_x(omega);
    }

    Eigen::Matrix3d Omega = skew_x(omega);
    return Eigen::Matrix3d::Identity() +
           (sin(theta) / theta) * Omega +
           ((1.0 - cos(theta)) / (theta * theta)) * Omega * Omega;
}


Eigen::Vector3d UpdaterWheel::log_so3(const Eigen::Matrix3d& R) {
    double theta = acos(std::min(1.0, std::max(-1.0, 0.5 * (R.trace() - 1.0))));

    if (theta < 1e-6) {
        return Eigen::Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1)) / 2.0;
    }

    return theta / (2.0 * sin(theta)) *
           Eigen::Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
}


Eigen::Matrix3d UpdaterWheel::skew_x(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}
*/
