#include "UpdaterWheel.h"
#include "utils/print.h"
#include "state/StateHelper.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace Eigen;


UpdaterWheel::UpdaterWheel(std::shared_ptr<State> state) : state(state) {
    PRINT_DEBUG("[WHEEL] Initializing UpdaterWheel...\n");
    last_updated_clone_time = -1.0;

    // Initialize preintegrated values
    delta_p.setZero();
    delta_R.setIdentity();
    covariance.setZero();

    // Default extrinsics (identity)
    T_imu_odom = Matrix4d::Identity();

    // Default noise parameters
    noise_gyro = 0.2;  // rad/s
    noise_vel = 0.5;   // m/s
    PRINT_DEBUG("[WHEEL] UpdaterWheel initialized. Noise params: gyro=%.4f, vel=%.4f\n", noise_gyro, noise_vel);
}


void UpdaterWheel::feed_measurement(const OdometryData& message, double oldest_time) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);
    odometry_data.push_back(message);
    PRINT_DEBUG("[WHEEL] ODOMETRY DATA PUSHLANDI \n");
    PRINT_DEBUG("[WHEEL] Feed: t=%.3f, lin=[%.3f,%.3f,%.3f], ang=[%.3f,%.3f,%.3f], buffer_size=%zu\n",
                message.timestamp,
                message.linear_velocity(0), message.linear_velocity(1), message.linear_velocity(2),
                message.angular_velocity(0), message.angular_velocity(1), message.angular_velocity(2),
                odometry_data.size());

    clean_old_measurements(oldest_time);
}


void UpdaterWheel::try_update() {
    // Check if we have valid clone times
    if (state->_clones_IMU.empty()) {
        PRINT_DEBUG("[WHEEL] try_update: No clones available\n");
        return;
    }

    // Get the oldest and newest clone times
    double oldest_time = state->_clones_IMU.begin()->first;
    double newest_time = state->_clones_IMU.rbegin()->first;
    PRINT_DEBUG("[WHEEL] try_update: clones [%.3f to %.3f], last_updated=%.3f, num_clones=%zu\n",
                oldest_time, newest_time, last_updated_clone_time, state->_clones_IMU.size());

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
        PRINT_DEBUG("[WHEEL] try_update: Attempting update [%.3f -> %.3f]\n",
                    last_updated_clone_time, it->first);

        // Try to update between last_updated_clone_time and current clone
        if (!update(last_updated_clone_time, it->first)) {
            PRINT_DEBUG("[WHEEL] try_update: Update failed, stopping\n");
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

        preintegration_RK4(dt, d1, d2);
    }

    // Compute linear system (H matrix and residuals)
    MatrixXd H;
    VectorXd res;
    std::vector<std::shared_ptr<ov_type::Type>> x_order;

    if (!compute_linear_system(H, res, x_order, time0, time1)) {
        PRINT_DEBUG("[WHEEL]: Failed to compute linear system\n");
        return false;
    }

    // Debug output
    PRINT_DEBUG("[WHEEL]: Update between %.3f and %.3f\n", time0, time1);
    PRINT_DEBUG("[WHEEL]: H size: %d x %d, res size: %d, cov size: %d x %d\n",
                (int)H.rows(), (int)H.cols(), (int)res.size(),
                (int)covariance.rows(), (int)covariance.cols());

    // Perform EKF update
    StateHelper::EKFUpdate(state, x_order, H, res, covariance);

    // Record last updated time
    last_updated_clone_time = time1;
    return true;
}


bool UpdaterWheel::select_odometry_data(double time0, double time1,
                                         std::vector<OdometryData>& data_vec) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);

    if (odometry_data.empty() || time1 <= time0) {
        PRINT_DEBUG("[WHEEL] select_odometry_data: Buffer is empty\n");
        return false;
    }

    // Check if we have data covering the time range
    if (odometry_data.back().timestamp < time1 || odometry_data.front().timestamp > time0) {
        PRINT_DEBUG("[WHEEL] select_odometry_data: Invalid time range [%.3f, %.3f]\n", time0, time1);
        return false;
    }
    PRINT_DEBUG("[WHEEL] select_odometry_data: Requested [%.3f, %.3f], buffer [%.3f, %.3f], size=%zu\n",
                time0, time1,
                odometry_data.front().timestamp, odometry_data.back().timestamp,
                odometry_data.size());

    // Find measurements within the time range
    bool found_start = false;

    for (size_t i = 0; i < odometry_data.size(); ++i) {
        const auto& curr = odometry_data[i];

        // Handle start boundary - interpolate if needed
        if (!found_start && i + 1 < odometry_data.size()) {
            const auto& next = odometry_data[i + 1];
            if (curr.timestamp <= time0 && next.timestamp > time0) {
                OdometryData interp = interpolate_data(curr, next, time0);
                data_vec.push_back(interp);
                found_start = true;
                PRINT_DEBUG("[WHEEL] select_odometry_data: Interpolated start at %.3f\n", time0);
                continue;
            }
        }

        // Add measurements in the middle
        if (curr.timestamp >= time0 && curr.timestamp <= time1) {
            data_vec.push_back(curr);
        }

        // Handle end boundary - interpolate if needed
        if (curr.timestamp < time1 && i + 1 < odometry_data.size()) {
            const auto& next = odometry_data[i + 1];
            if (next.timestamp >= time1 && curr.timestamp < time1) {
                OdometryData interp = interpolate_data(curr, next, time1);
                data_vec.push_back(interp);
                PRINT_DEBUG("[WHEEL] select_odometry_data: Interpolated end at %.3f\n", time1);
                break;
            }
        }

        // Stop if we've passed time1
        if (curr.timestamp > time1) {
            break;
        }
    }

    PRINT_DEBUG("[WHEEL] select_odometry_data: %zu measurements selected\n", data_vec.size());

    return data_vec.size() >= 2;
}


void UpdaterWheel::clean_old_measurements(double oldest_time) {
    if (oldest_time < 0){
        PRINT_DEBUG("[WHEEL] clean_old_measurements failed \n");
        return;
    }

    auto it = odometry_data.begin();
    while (it != odometry_data.end()) {
        // Keep a buffer of older measurements for interpolation
        if (it->timestamp < oldest_time - 2.0) {
            it = odometry_data.erase(it);
        } else {
            ++it;
        }
    }
}

void UpdaterWheel::preintegration_RK4(double dt, const OdometryData& data1, const OdometryData& data2) {
    // Get angular and linear velocities from odometry
    Vector3d w1 = data1.angular_velocity;
    Vector3d v1 = data1.linear_velocity;
    Vector3d w2 = data2.angular_velocity;
    Vector3d v2 = data2.linear_velocity;

    // Transform velocities from odometry frame to IMU frame using extrinsics
    Matrix3d R_ItoO = T_imu_odom.block<3,3>(0,0);
    Vector3d p_IinO = T_imu_odom.block<3,1>(0,3);

    // Transform to IMU frame
    Vector3d w1_imu = R_ItoO.transpose() * w1;
    Vector3d v1_imu = R_ItoO.transpose() * (v1 - w1.cross(p_IinO));
    Vector3d w2_imu = R_ItoO.transpose() * w2;
    Vector3d v2_imu = R_ItoO.transpose() * (v2 - w2.cross(p_IinO));

    // ============ RK4 Integration ============
    // Current state
    Matrix3d R0 = delta_R;
    Vector3d p0 = delta_p;

    // k1 calculation (at t0)
    Vector3d w_k1 = w1_imu;
    Vector3d v_k1 = v1_imu;
    Matrix3d dR_k1 = R0 * exp_so3(w_k1 * dt);
    Vector3d dp_k1 = R0 * v_k1 * dt;

    // k2 calculation (at t0 + dt/2)
    Vector3d w_k2 = 0.5 * (w1_imu + w2_imu);
    Vector3d v_k2 = 0.5 * (v1_imu + v2_imu);
    Matrix3d R_k2 = R0 * exp_so3(w_k1 * dt * 0.5);
    Matrix3d dR_k2 = R_k2 * exp_so3(w_k2 * dt);
    Vector3d dp_k2 = R_k2 * v_k2 * dt;

    // k3 calculation (at t0 + dt/2, using k2 slope)
    Matrix3d R_k3 = R0 * exp_so3(w_k2 * dt * 0.5);
    Matrix3d dR_k3 = R_k3 * exp_so3(w_k2 * dt);
    Vector3d dp_k3 = R_k3 * v_k2 * dt;

    // k4 calculation (at t0 + dt)
    Vector3d w_k4 = w2_imu;
    Vector3d v_k4 = v2_imu;
    Matrix3d R_k4 = R0 * exp_so3(w_k2 * dt);
    Matrix3d dR_k4 = R_k4 * exp_so3(w_k4 * dt);
    Vector3d dp_k4 = R_k4 * v_k4 * dt;

    // Weighted average (RK4 formula)
    // For rotation: use composition on manifold
    Vector3d w_avg_rk4 = (w_k1 + 2.0*w_k2 + 2.0*w_k2 + w_k4) / 6.0;
    Matrix3d R_new = R0 * exp_so3(w_avg_rk4 * dt);

    // For position: weighted average of increments
    Vector3d p_new = p0 + (dp_k1 + 2.0*dp_k2 + 2.0*dp_k3 + dp_k4) / 6.0;

    // ============ Covariance Propagation ============
    // Use the average angular velocity for Jacobian computation
    Vector3d w_avg = (w1_imu + w2_imu) * 0.5;
    Vector3d v_avg = (v1_imu + v2_imu) * 0.5;

    // Compute Jacobians using average values
    Matrix3d dR_avg = exp_so3(w_avg * dt);

    Matrix<double, 6, 6> F = Matrix<double, 6, 6>::Identity();
    F.block<3,3>(0,0) = dR_avg.transpose();
    F.block<3,3>(3,0) = -delta_R * skew_x(v_avg * dt);

    // Noise Jacobian
    Matrix<double, 6, 6> G = Matrix<double, 6, 6>::Zero();
    G.block<3,3>(0,0) = delta_R * dt;
    G.block<3,3>(3,3) = delta_R * dt;

    // Process noise covariance
    Matrix<double, 6, 6> Q = Matrix<double, 6, 6>::Zero();
    Q.block<3,3>(0,0) = (noise_gyro * noise_gyro / dt) * Matrix3d::Identity();
    Q.block<3,3>(3,3) = (noise_vel * noise_vel / dt) * Matrix3d::Identity();

    // Propagate covariance
    covariance = F * covariance * F.transpose() + G * Q * G.transpose();

    // Ensure symmetry
    covariance = 0.5 * (covariance + covariance.transpose());

    // Update preintegrated values
    delta_R = R_new;
    delta_p = p_new;

    PRINT_DEBUG("[WHEEL] preintegrate_RK4: dt=%.4f, delta_p=[%.3f,%.3f,%.3f], delta_R_norm=%.6f\n",
                dt, delta_p(0), delta_p(1), delta_p(2),
                log_so3(delta_R).norm());
}

void UpdaterWheel::preintegration_3D(double dt, const OdometryData& data1, const OdometryData& data2) {
    // Get angular and linear velocities from odometry
    // Average the velocities for better integration (trapezoidal rule)
    Vector3d w1 = data1.angular_velocity;
    Vector3d v1 = data1.linear_velocity;
    Vector3d w2 = data2.angular_velocity;
    Vector3d v2 = data2.linear_velocity;

    // Average velocities
    Vector3d w_avg = 0.5 * (w1 + w2);
    Vector3d v_avg = 0.5 * (v1 + v2);

    // Transform velocities from odometry frame to IMU frame using extrinsics
    Matrix3d R_ItoO = T_imu_odom.block<3,3>(0,0);
    Vector3d p_IinO = T_imu_odom.block<3,1>(0,3);

    Vector3d w_imu = R_ItoO.transpose() * w_avg;
    Vector3d v_imu = R_ItoO.transpose() * (v_avg - w_avg.cross(p_IinO));

    // Preintegrate rotation using exponential map
    Matrix3d dR = exp_so3(w_imu * dt);
    Matrix3d R_new = delta_R * dR;

    // Preintegrate position
    Vector3d p_new = delta_p + delta_R * v_imu * dt;

    // Compute Jacobians for covariance propagation
    Matrix<double, 6, 6> F = Matrix<double, 6, 6>::Identity();
    F.block<3,3>(0,0) = dR.transpose();
    F.block<3,3>(3,0) = -delta_R * skew_x(v_imu * dt);

    // Noise Jacobian
    Matrix<double, 6, 6> G = Matrix<double, 6, 6>::Zero();
    G.block<3,3>(0,0) = delta_R * dt;
    G.block<3,3>(3,3) = delta_R * dt;

    // Process noise covariance
    Matrix<double, 6, 6> Q = Matrix<double, 6, 6>::Zero();
    Q.block<3,3>(0,0) = (noise_gyro * noise_gyro / dt) * Matrix3d::Identity();
    Q.block<3,3>(3,3) = (noise_vel * noise_vel / dt) * Matrix3d::Identity();

    // Propagate covariance
    covariance = F * covariance * F.transpose() + G * Q * G.transpose();

    // Ensure symmetry
    covariance = 0.5 * (covariance + covariance.transpose());

    // Update preintegrated values
    delta_R = R_new;
    delta_p = p_new;

    PRINT_DEBUG("[WHEEL] preintegrate: dt=%.4f, delta_p=[%.3f,%.3f,%.3f], delta_R_norm=%.6f\n",
            dt, delta_p(0), delta_p(1), delta_p(2),
            log_so3(delta_R).norm());
}


bool UpdaterWheel::compute_linear_system(MatrixXd& H, VectorXd& res,
                                          std::vector<std::shared_ptr<ov_type::Type>>& x_order,
                                          double time0, double time1) {
    // Check if clones exist
    if (state->_clones_IMU.find(time0) == state->_clones_IMU.end() ||
        state->_clones_IMU.find(time1) == state->_clones_IMU.end()) {
        PRINT_DEBUG("[WHEEL]: Clones not found for times %.3f and %.3f\n", time0, time1);
        return false;
    }

    PRINT_DEBUG("[WHEEL]: Clones found for times %.3f and %.3f\n", time0, time1);

    // Get clone poses
    auto clone0 = state->_clones_IMU.at(time0);
    auto clone1 = state->_clones_IMU.at(time1);

    // Extract poses (use FEJ if needed, but for now use current values)
    Matrix3d R_GtoI0 = clone0->Rot();
    Vector3d p_I0inG = clone0->pos();
    Matrix3d R_GtoI1 = clone1->Rot();
    Vector3d p_I1inG = clone1->pos();

    // Transform to odometry frame
    Matrix3d R_ItoO = T_imu_odom.block<3,3>(0,0);
    Vector3d p_IinO = T_imu_odom.block<3,1>(0,3);
    Vector3d p_OinI = -R_ItoO.transpose() * p_IinO;

    // Compute expected relative transformation in odometry frame
    Matrix3d R_I0toI1 = R_GtoI1 * R_GtoI0.transpose();
    Matrix3d R_O0toO1_expected = R_ItoO * R_I0toI1 * R_ItoO.transpose();

    Vector3d p_I1inI0 = R_GtoI0 * (p_I1inG - p_I0inG);
    Vector3d p_O1inO0_expected = R_ItoO * (p_I1inI0 + R_I0toI1 * p_OinI - p_OinI);

    // Compute residuals (measurement - estimate)
    res = VectorXd::Zero(6);

    // Rotation residual: log(delta_R * R_expected^T)
    Matrix3d R_err = delta_R * R_O0toO1_expected.transpose();
    res.segment<3>(0) = log_so3(R_err);

    // Position residual
    res.segment<3>(3) = delta_p - p_O1inO0_expected;

    // ============================================
    // CRITICAL: StateHelper::EKFUpdate expects H matrix with columns = sum of Type::size()
    // For PoseJPL: size() = 7 (4 quat + 3 pos)
    // We have 2 clones, so H should be 6 x 14
    // ============================================

    // STEP 1: First create x_order (IMPORTANT: Do this BEFORE calculating H size)
    x_order.clear();
    x_order.push_back(clone0);
    x_order.push_back(clone1);

    // STEP 2: Calculate total H columns from x_order
    int total_hx = 0;
    for (const auto& var : x_order) {
        total_hx += var->size();  // This will be 7 + 7 = 14
    }

    PRINT_DEBUG("[WHEEL] Clone0: id=%d, size=%d\n", clone0->id(), clone0->size());
    PRINT_DEBUG("[WHEEL] Clone1: id=%d, size=%d\n", clone1->id(), clone1->size());
    PRINT_DEBUG("[WHEEL] Total H columns (from x_order): %d\n", total_hx);

    // STEP 3: Create H matrix with CORRECT size
    H = MatrixXd::Zero(6, total_hx);  // Should be 6 x 14

    // Get Right Jacobian inverse for rotation error
    Vector3d w_err = log_so3(R_err);
    Matrix3d Jr_inv = Jr_so3_inv(w_err);

    // ========== Compute Jacobians ==========
    // Rotation residual Jacobians
    Matrix3d H_rot_R0 = Jr_inv * R_ItoO * R_GtoI1;
    Matrix3d H_rot_R1 = -Jr_inv * R_ItoO;

    // Position residual Jacobians
    Matrix3d H_pos_R0 = R_ItoO * skew_x(R_GtoI0 * (p_I1inG - p_I0inG));
    Matrix3d H_pos_p0 = -R_ItoO * R_GtoI0;
    Matrix3d H_pos_R1 = -R_ItoO * R_GtoI0 * R_GtoI1.transpose() * skew_x(p_OinI);
    Matrix3d H_pos_p1 = R_ItoO * R_GtoI0;

    // ========== Fill H matrix ==========
    // IMPORTANT: PoseJPL internal state is [quat(4), pos(3)] = 7 elements
    // But on manifold it's [rotation(3), position(3)] = 6 dof
    // StateHelper::EKFUpdate uses size() which returns 7!

    // Clone0 occupies first 7 columns (indices 0-6)
    // We fill the manifold Jacobian (6 dof) into these 7 columns
    // The 4th column (quaternion w component) remains zero

    int clone0_size = clone0->size();  // Should be 7
    int clone1_size = clone1->size();  // Should be 7

    // Clone0 Jacobians (columns 0-6, but only using first 6 dof)
    H.block<3,3>(0, 0) = H_rot_R0;  // Rotation residual wrt R0 (cols 0-2)
    H.block<3,3>(3, 0) = H_pos_R0;  // Position residual wrt R0 (cols 0-2)
    H.block<3,3>(3, 3) = H_pos_p0;  // Position residual wrt p0 (cols 3-5)
    // Column 6 (7th column) for quat w component stays zero

    // Clone1 Jacobians (columns 7-13, but only using first 6 dof)
    int idx1 = clone0_size;  // Start at column 7
    H.block<3,3>(0, idx1+0) = H_rot_R1;  // Rotation residual wrt R1 (cols 7-9)
    H.block<3,3>(3, idx1+0) = H_pos_R1;  // Position residual wrt R1 (cols 7-9)
    H.block<3,3>(3, idx1+3) = H_pos_p1;  // Position residual wrt p1 (cols 10-12)
    // Column 13 (14th column) for quat w component stays zero

    PRINT_DEBUG("[WHEEL] Residuals: rot_err=[%.4f,%.4f,%.4f] (norm=%.4f), pos_err=[%.4f,%.4f,%.4f] (norm=%.4f)\n",
                res(0), res(1), res(2), res.segment<3>(0).norm(),
                res(3), res(4), res(5), res.segment<3>(3).norm());

    PRINT_DEBUG("[WHEEL] Covariance diagonal: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
                covariance(0,0), covariance(1,1), covariance(2,2),
                covariance(3,3), covariance(4,4), covariance(5,5));

    PRINT_DEBUG("[WHEEL] H matrix: %dx%d, condition_number=%.2e\n",
                (int)H.rows(), (int)H.cols(),
                H.norm() / (H.completeOrthogonalDecomposition().pseudoInverse().norm() + 1e-10));

    // Sanity checks
    assert(H.rows() == 6);
    assert(H.cols() == total_hx);
    assert(res.rows() == 6);
    assert(covariance.rows() == 6 && covariance.cols() == 6);

    PRINT_DEBUG("[WHEEL] compute linear system complete - all checks passed\n");

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
    PRINT_DEBUG("[WHEEL] interpolate_data \n");

    return interp;
}


// ========== Utility Functions ==========

Matrix3d UpdaterWheel::exp_so3(const Vector3d& omega) {
    double theta = omega.norm();

    if (theta < 1e-8) {
        return Matrix3d::Identity() + skew_x(omega);
    }

    Matrix3d Omega = skew_x(omega);
    return Matrix3d::Identity() +
           (sin(theta) / theta) * Omega +
           ((1.0 - cos(theta)) / (theta * theta)) * Omega * Omega;
}


Vector3d UpdaterWheel::log_so3(const Matrix3d& R) {
    double theta = acos(std::min(1.0, std::max(-1.0, 0.5 * (R.trace() - 1.0))));

    if (theta < 1e-8) {
        return Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1)) / 2.0;
    }

    return theta / (2.0 * sin(theta)) *
           Vector3d(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
}


Matrix3d UpdaterWheel::skew_x(const Vector3d& v) {
    Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}


Matrix3d UpdaterWheel::Jr_so3(const Vector3d& w) {
    double theta = w.norm();

    if (theta < 1e-8) {
        return Matrix3d::Identity() - 0.5 * skew_x(w);
    }

    Matrix3d W = skew_x(w);
    return Matrix3d::Identity()
           - ((1.0 - cos(theta))/(theta*theta)) * W
           + ((theta - sin(theta))/(theta*theta*theta)) * W * W;
}


Matrix3d UpdaterWheel::Jr_so3_inv(const Vector3d& w) {
    double theta = w.norm();

    if (theta < 1e-8) {
        return Matrix3d::Identity() + 0.5 * skew_x(w);
    }

    Matrix3d W = skew_x(w);
    return Matrix3d::Identity()
           + 0.5 * W
           + (1.0/(theta*theta) - (1.0 + cos(theta))/(2.0*theta*sin(theta))) * W * W;
}


Vector4d UpdaterWheel::rot_2_quat(const Matrix3d& R) {
    Vector4d q;
    double T = R.trace();

    if (T > 0) {
        double S = sqrt(T + 1.0) * 2.0;
        q(0) = (R(2,1) - R(1,2)) / S;
        q(1) = (R(0,2) - R(2,0)) / S;
        q(2) = (R(1,0) - R(0,1)) / S;
        q(3) = 0.25 * S;
    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
        double S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2.0;
        q(0) = 0.25 * S;
        q(1) = (R(0,1) + R(1,0)) / S;
        q(2) = (R(0,2) + R(2,0)) / S;
        q(3) = (R(2,1) - R(1,2)) / S;
    } else if (R(1,1) > R(2,2)) {
        double S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2.0;
        q(0) = (R(0,1) + R(1,0)) / S;
        q(1) = 0.25 * S;
        q(2) = (R(1,2) + R(2,1)) / S;
        q(3) = (R(0,2) - R(2,0)) / S;
    } else {
        double S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2.0;
        q(0) = (R(0,2) + R(2,0)) / S;
        q(1) = (R(1,2) + R(2,1)) / S;
        q(2) = 0.25 * S;
        q(3) = (R(1,0) - R(0,1)) / S;
    }

    return q / q.norm();
}


Matrix3d UpdaterWheel::quat_2_Rot(const Vector4d& q) {
    Matrix3d R;
    double qx = q(0), qy = q(1), qz = q(2), qw = q(3);

    R(0,0) = 1 - 2*qy*qy - 2*qz*qz;
    R(0,1) = 2*qx*qy - 2*qz*qw;
    R(0,2) = 2*qx*qz + 2*qy*qw;

    R(1,0) = 2*qx*qy + 2*qz*qw;
    R(1,1) = 1 - 2*qx*qx - 2*qz*qz;
    R(1,2) = 2*qy*qz - 2*qx*qw;

    R(2,0) = 2*qx*qz - 2*qy*qw;
    R(2,1) = 2*qy*qz + 2*qx*qw;
    R(2,2) = 1 - 2*qx*qx - 2*qy*qy;

    return R;
}


Vector4d UpdaterWheel::quat_multiply(const Vector4d& q1, const Vector4d& q2) {
    Vector4d q;
    q(0) = q1(3)*q2(0) + q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1);
    q(1) = q1(3)*q2(1) - q1(0)*q2(2) + q1(1)*q2(3) + q1(2)*q2(0);
    q(2) = q1(3)*q2(2) + q1(0)*q2(1) - q1(1)*q2(0) + q1(2)*q2(3);
    q(3) = q1(3)*q2(3) - q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2);
    return q;
}


Vector4d UpdaterWheel::quatnorm(const Vector4d& q) {
    return q / q.norm();
}


Matrix4d UpdaterWheel::Omega(const Vector3d& w) {
    Matrix4d Omega;
    Omega << 0, w(2), -w(1), w(0),
             -w(2), 0, w(0), w(1),
             w(1), -w(0), 0, w(2),
             -w(0), -w(1), -w(2), 0;
    return Omega;
}
