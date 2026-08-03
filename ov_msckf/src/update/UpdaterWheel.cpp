#include "UpdaterWheel.h"
#include "utils/print.h"
#include "utils/colors.h"
#include "state/StateHelper.h"
#include "utils/sensor_data.h"

#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_msckf;
using namespace ov_core;
using namespace Eigen;


UpdaterWheel::UpdaterWheel(std::shared_ptr<State> state) : state(state) {
    last_updated_clone_time = -1.0;
    delta_p.setZero();
    delta_R.setIdentity();
    covariance.setZero();

    // Precompute chi2 95% thresholds (same as UpdaterMSCKF)
    for (int i = 1; i < 500; i++) {
        boost::math::chi_squared chi_squared_dist(i);
        chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }
}


void UpdaterWheel::feed_measurement(const OdometryData& message, double oldest_time) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);
    odometry_data.push_back(message);
    clean_old_measurements(oldest_time);
}


void UpdaterWheel::try_update() {
    if (state->_clones_IMU.empty()) {
        PRINT_DEBUG("[WHEEL] try_update: No clones available\n");
        return;
    }

    double oldest_time = state->_clones_IMU.begin()->first;
    double newest_time = state->_clones_IMU.rbegin()->first;
    PRINT_DEBUG("[WHEEL] try_update: clones [%.3f to %.3f], last_updated=%.3f, num_clones=%zu\n",
                oldest_time, newest_time, last_updated_clone_time, state->_clones_IMU.size());

    // Initialize or recover last_updated_clone_time
    if (last_updated_clone_time < 0 ||
        state->_clones_IMU.find(last_updated_clone_time) == state->_clones_IMU.end()) {
        last_updated_clone_time = oldest_time;
        return;
    }

    double newest_clone_time = state->_clones_IMU.rbegin()->first;

    for (auto it = state->_clones_IMU.begin(); it != state->_clones_IMU.end(); ++it) {
        if (it->first <= last_updated_clone_time)
            continue;

        PRINT_DEBUG("[WHEEL] try_update: Attempting update [%.3f -> %.3f]\n",
                    last_updated_clone_time, it->first);

        if (!update(last_updated_clone_time, it->first)) {
            PRINT_DEBUG("[WHEEL] try_update: Update failed\n");
            // En yeni pencere için veri henüz gelmemiş olabilir — bekle.
            // Eski pencereler için veri kalıcı olarak eksikse takılmamak adına atla.
            if (it->first < newest_clone_time) {
                last_updated_clone_time = it->first;
                continue;
            }
            break;
        }
    }
}


bool UpdaterWheel::update(double time0, double time1) {
    std::vector<OdometryData> data_vec;
    if (!select_odometry_data(time0, time1, data_vec) || data_vec.size() < 2) {
        PRINT_DEBUG("[WHEEL]: Not enough odometry measurements between %.3f and %.3f\n", time0, time1);
        return false;
    }

    // Turn detection: penceredeki herhangi bir ölçümde yüksek açısal hız varsa atla.
    // NOT: try_update'teki eski kontrol odometry_data.back() kullanıyordu — bu, şu anki
    // mesajın pencerenin dışında olmasına rağmen tüm güncellemeleri blokluyordu.
    if (turn_detection_enabled) {
        for (const auto& d : data_vec) {
            if (d.angular_velocity.norm() > turn_ang_threshold) {
                last_was_pure_rotation = true;
                PRINT_DEBUG("[WHEEL] Turn detected in window [%.3f,%.3f] w=%.3f > %.3f — skipping\n",
                            time0, time1, d.angular_velocity.norm(), turn_ang_threshold);
                last_updated_clone_time = time1;
                return true;
            }
        }
        last_was_pure_rotation = false;
    }

    // Reset preintegration — always starts from identity/zero
    delta_p.setZero();
    delta_R.setIdentity();
    covariance.setZero();

    for (size_t i = 0; i + 1 < data_vec.size(); ++i) {
        const auto& d1 = data_vec[i];
        const auto& d2 = data_vec[i + 1];
        double dt = d2.timestamp - d1.timestamp;
        if (dt <= 1e-6) {
            PRINT_DEBUG("[WHEEL] WARNING: dt too small (%.9f), skipping\n", dt);
            continue;
        }
        preintegration_RK4(dt, d1, d2);
    }

    PRINT_DEBUG("[WHEEL] Preintegration complete: delta_p=[%.4f,%.4f,%.4f], delta_R_angle=%.4f rad\n",
                delta_p(0), delta_p(1), delta_p(2), log_so3(delta_R).norm());

    MatrixXd H;
    VectorXd res;
    std::vector<std::shared_ptr<ov_type::Type>> x_order;

    if (!compute_linear_system(H, res, x_order, time0, time1)) {
        PRINT_DEBUG("[WHEEL]: Failed to compute linear system\n");
        return false;
    }

    PRINT_DEBUG("[WHEEL] res=[%.4f,%.4f,%.4f | %.4f,%.4f,%.4f], cov_trace=%.6f\n",
                res(0),res(1),res(2),res(3),res(4),res(5), covariance.trace());

    // Platform-aware adaptive measurement covariance (per-axis correntropy +
    // sliding-window R estimate + gait handling). Applied BEFORE the chi2 gate
    // so the gate sees the same R the EKF will use: an inflated R already
    // softens an outlier, and gating on the raw R would penalise it twice.
    Matrix<double, 6, 6> R_eff = covariance;
    if (platform != nullptr && res.rows() == 6) {
        Matrix<double, 6, 1> res6 = res.head<6>();
        platform->adapt_R(R_eff, res6);
        platform->push_innovation(time1, res6);
    }

    MatrixXd P_marg = StateHelper::get_marginal_covariance(state, x_order);
    MatrixXd S_check = H * P_marg * H.transpose() + R_eff;
    PRINT_DEBUG("[WHEEL] P_marg trace=%.6f, H*P*Ht trace=%.6f, R trace=%.6f\n",
                P_marg.trace(), (H * P_marg * H.transpose()).trace(), R_eff.trace());
    PRINT_DEBUG("[WHEEL] S trace=%.6f, S det=%.6e\n",
                S_check.trace(), S_check.determinant());


    // Chi2 outlier rejection — same pattern as UpdaterMSCKF / MINS Chi2Check
    MatrixXd P_marg_chi2 = StateHelper::get_marginal_covariance(state, x_order);
    MatrixXd S = H * P_marg_chi2 * H.transpose() + R_eff;
    double chi2 = res.dot(S.llt().solve(res));

    double chi2_check;
    if (res.rows() < 500) {
        chi2_check = chi_squared_table[res.rows()];
    } else {
        boost::math::chi_squared chi_squared_dist(res.rows());
        chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    }

    if (chi2 > chi2_mult * chi2_check) {
        PRINT_WARNING(YELLOW "[WHEEL] Chi2 FAILED: %.3f > %.3f (mult=%.1f, dof=%d) — skipping\n" RESET,
                      chi2, chi2_mult * chi2_check, chi2_mult, (int)res.rows());
        last_updated_clone_time = time1;
        return true;
    }
    PRINT_DEBUG("[WHEEL] Chi2 passed: %.3f < %.3f\n", chi2, chi2_mult * chi2_check);
    PRINT_INFO("[WHEEL] EKFUpdate called count=%d\n", ++update_count);
    StateHelper::EKFUpdate(state, x_order, H, res, R_eff);


    // EKFUpdate'ten SONRA residual'ı tekrar hesapla
    MatrixXd H2;
    VectorXd res2;
    std::vector<std::shared_ptr<ov_type::Type>> x_order2;
    compute_linear_system(H2, res2, x_order2, time0, time1);
    PRINT_DEBUG("[WHEEL] POST-UPDATE residual: rot=[%.4f,%.4f,%.4f] pos=[%.4f,%.4f,%.4f]\n",
            res2(0),res2(1),res2(2),res2(3),res2(4),res2(5));
    last_updated_clone_time = time1;
    return true;
}


bool UpdaterWheel::select_odometry_data(double time0, double time1,
                                         std::vector<OdometryData>& data_vec) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);

    if (odometry_data.empty() || time1 <= time0) {
        PRINT_DEBUG("[WHEEL] select_odometry_data: Buffer empty or invalid range\n");
        return false;
    }

    if (odometry_data.back().timestamp < time1 || odometry_data.front().timestamp > time0) {
        PRINT_DEBUG("[WHEEL] select_odometry_data: Buffer [%.3f, %.3f] does not cover [%.3f, %.3f]\n",
                    odometry_data.front().timestamp, odometry_data.back().timestamp, time0, time1);
        return false;
    }

    bool found_start = false;

    for (size_t i = 0; i < odometry_data.size(); ++i) {
        const auto& curr = odometry_data[i];

        // Interpolate at start boundary
        if (!found_start && i + 1 < odometry_data.size()) {
            const auto& next = odometry_data[i + 1];
            if (curr.timestamp <= time0 && next.timestamp > time0) {
                data_vec.push_back(interpolate_data(curr, next, time0));
                found_start = true;
                continue;
            }
        }

        // Add measurements strictly inside the range
        if (curr.timestamp >= time0 && curr.timestamp <= time1) {
            data_vec.push_back(curr);
        }

        // Interpolate at end boundary
        if (curr.timestamp < time1 && i + 1 < odometry_data.size()) {
            const auto& next = odometry_data[i + 1];
            if (next.timestamp >= time1) {
                data_vec.push_back(interpolate_data(curr, next, time1));
                break;
            }
        }

        if (curr.timestamp > time1)
            break;
    }

    return data_vec.size() >= 2;
}


void UpdaterWheel::clean_old_measurements(double oldest_time) {
    if (oldest_time < 0)
        return;

    auto it = odometry_data.begin();
    while (it != odometry_data.end()) {
        if (it->timestamp < oldest_time - 2.0)
            it = odometry_data.erase(it);
        else
            ++it;
    }
}


void UpdaterWheel::preintegration_RK4(double dt, const OdometryData& data1, const OdometryData& data2) {

    // -----------------------------------------------------------------------
    // IMPORTANT FRAME CONVENTION:
    //   All preintegration is done in the ODOMETRY frame (same as MINS).
    //   delta_R = R_O0toO1  (rotation of O1 expressed in O0 frame)
    //   delta_p = p_O1inO0  (position of O1 origin expressed in O0 frame)
    //
    //   The odometry message already provides velocities in the odometry frame:
    //     v = linear velocity of odometry origin, expressed in odometry frame
    //     w = angular velocity of odometry frame, expressed in odometry frame
    //
    //   We do NOT transform to IMU frame here. The state Jacobians in
    //   compute_linear_system handle the IMU<->Odometry extrinsic transform.
    // -----------------------------------------------------------------------
    PRINT_DEBUG("[WHEEL] RAW odom: lin=[%.4f,%.4f,%.4f] ang=[%.4f,%.4f,%.4f]\n",
                data1.linear_velocity(0), data1.linear_velocity(1), data1.linear_velocity(2),
                data1.angular_velocity(0), data1.angular_velocity(1), data1.angular_velocity(2));
    Vector3d w1 = data1.angular_velocity;  // in odometry frame
    Vector3d v1 = data1.linear_velocity;   // in odometry frame
    Vector3d w2 = data2.angular_velocity;
    Vector3d v2 = data2.linear_velocity;

    // Current preintegrated state
    Matrix3d R0 = delta_R;
    Vector3d p0 = delta_p;

    // Angular acceleration and linear jerk for RK4 interpolation
    Vector3d w_alpha = (w2 - w1) / dt;
    Vector3d v_jerk  = (v2 - v1) / dt;

    // Use quaternion representation matching MINS RK4 style
    Vector4d q_local = rot_2_quat(R0);
    Vector4d dq_0 = {0, 0, 0, 1};  // identity increment

    // ---- k1 (at t0) ----
    Vector4d q0_dot = 0.5 * Omega(w1) * dq_0;
    Matrix3d R_at_k1 = quat_2_Rot(quat_multiply(dq_0, q_local));
    Vector4d k1_q = q0_dot * dt;
    Vector3d k1_p = R_at_k1.transpose() * v1 * dt;

    // ---- k2 (at t0 + dt/2) ----
    Vector3d w_mid = w1 + 0.5 * w_alpha * dt;
    Vector3d v_mid = v1 + 0.5 * v_jerk  * dt;
    Vector4d dq_1  = quatnorm(dq_0 + 0.5 * k1_q);
    Vector4d q1_dot = 0.5 * Omega(w_mid) * dq_1;
    Matrix3d R_at_k2 = quat_2_Rot(quat_multiply(dq_1, q_local));
    Vector4d k2_q = q1_dot * dt;
    Vector3d k2_p = R_at_k2.transpose() * v_mid * dt;

    // ---- k3 (at t0 + dt/2, using k2 slope) ----
    Vector4d dq_2  = quatnorm(dq_0 + 0.5 * k2_q);
    Vector4d q2_dot = 0.5 * Omega(w_mid) * dq_2;
    Matrix3d R_at_k3 = quat_2_Rot(quat_multiply(dq_2, q_local));
    Vector4d k3_q = q2_dot * dt;
    Vector3d k3_p = R_at_k3.transpose() * v_mid * dt;

    // ---- k4 (at t0 + dt) ----
    Vector4d dq_3  = quatnorm(dq_0 + k3_q);
    Vector4d q3_dot = 0.5 * Omega(w2) * dq_3;
    Matrix3d R_at_k4 = quat_2_Rot(quat_multiply(dq_3, q_local));
    Vector4d k4_q = q3_dot * dt;
    Vector3d k4_p = R_at_k4.transpose() * v2 * dt;

    // ---- RK4 weighted average ----
    Vector4d dq   = quatnorm(dq_0 + (1.0/6.0)*k1_q + (1.0/3.0)*k2_q + (1.0/3.0)*k3_q + (1.0/6.0)*k4_q);
    Vector4d new_q = quat_multiply(dq, q_local);
    Matrix3d R_new = quat_2_Rot(new_q);
    Vector3d new_p = p0 + (1.0/6.0)*k1_p + (1.0/3.0)*k2_p + (1.0/3.0)*k3_p + (1.0/6.0)*k4_p;

    // -----------------------------------------------------------------------
    // Covariance propagation (matches MINS Phi_tr / Phi_ns pattern)
    //
    //   State vector: [delta_theta (3), delta_p (3)]
    //   Noise vector: [n_w (3), n_v (3)]  — in odometry body frame
    //
    //   Transition Jacobian F (how current error maps forward):
    //     d(delta_R_new)/d(delta_R) = R_new * R0^T   (rotation composition)
    //     d(delta_p_new)/d(delta_R) = -R0^T * skew(new_p - p0)
    //                               (how a rotation error rotates the position increment)
    //     d(delta_p_new)/d(delta_p) = I
    //
    //   Noise Jacobian G (how noise enters):
    //     Rotation noise enters as: Jl * n_w * dt  ≈  I * dt  (for small dt)
    //     Position noise enters as: R0^T * n_v * dt  (velocity noise in body frame)
    // -----------------------------------------------------------------------

    Matrix<double, 6, 6> F = Matrix<double, 6, 6>::Zero();
    F.block<3,3>(0, 0) = R_new * R0.transpose();
    F.block<3,3>(3, 0) = -R0.transpose() * skew_x(new_p - p0);
    F.block<3,3>(3, 3) = Matrix3d::Identity();

    Matrix<double, 6, 6> G = Matrix<double, 6, 6>::Zero();
    G.block<3,3>(0, 0) = Matrix3d::Identity() * dt;  // rotation noise
    G.block<3,3>(3, 3) = R0.transpose() * dt;         // velocity noise in body frame

    // Measurement noise on the odometry channel. Two ways of saying the same
    // thing, and the platform model is the more general one:
    //
    //   platform ON  → Q = base_noise(dt), one sigma per body axis taken from
    //                  noise_w_axis / noise_v_axis. This is where the vehicle's
    //                  kinematics physically enter the filter: for a car the
    //                  reported zeros on v_y / v_z are the nonholonomic
    //                  constraint, so those axes are TIGHTENED, and a legged
    //                  platform instead leaves them comparable to x.
    //
    //   platform OFF → the ackermann-specific scheme below (MINS Wheel3DAng
    //                  pattern), which hard-codes the same idea for one
    //                  morphology: encoder-derived yaw rate and forward
    //                  velocity get their own sigma, the four near-zero axes
    //                  share noise_pos.
    Matrix<double, 6, 6> Q;
    if (platform != nullptr) {
        Q = platform->base_noise(dt);
    } else {
        Q.setZero();
        Q(0, 0) = (noise_pos  * noise_pos  / dt);  // angular.x
        Q(1, 1) = (noise_pos  * noise_pos  / dt);  // angular.y
        Q(2, 2) = (noise_gyro * noise_gyro / dt);  // angular.z ← yaw rate
        Q(3, 3) = (noise_vel  * noise_vel  / dt);  // linear.x  ← forward
        Q(4, 4) = (noise_pos  * noise_pos  / dt);  // linear.y
        Q(5, 5) = (noise_pos  * noise_pos  / dt);  // linear.z
    }

    covariance = F * covariance * F.transpose() + G * Q * G.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());  // enforce symmetry

    // Update state
    delta_R = R_new;
    delta_p = new_p;
}


bool UpdaterWheel::compute_linear_system(MatrixXd& H, VectorXd& res,
                                          std::vector<std::shared_ptr<ov_type::Type>>& x_order,
                                          double time0, double time1) {

    if (state->_clones_IMU.find(time0) == state->_clones_IMU.end() ||
        state->_clones_IMU.find(time1) == state->_clones_IMU.end()) {
        PRINT_DEBUG("[WHEEL]: Clones not found for times %.3f and %.3f\n", time0, time1);
        return false;
    }

    auto clone0 = state->_clones_IMU.at(time0);
    auto clone1 = state->_clones_IMU.at(time1);

    PRINT_DEBUG("[WHEEL] clone0 size=%d id=%d, clone1 size=%d id=%d\n",
                (int)clone0->size(), (int)clone0->id(),
                (int)clone1->size(), (int)clone1->id());
    // T_imu_wheel = [R_OtoI | p_OinI ; 0 0 0 1]  ("from wheel to IMU", standard SE3)
    //   R_OtoI  : rotation from wheel/odometry frame to IMU frame
    //   p_OinI  : position of wheel/odometry origin in IMU frame (translation column)
    Matrix3d R_OtoI = T_imu_odom.block<3,3>(0,0);
    Matrix3d R_ItoO = R_OtoI.transpose();   // rotation from IMU to odometry frame
    Vector3d p_OinI = T_imu_odom.block<3,1>(0,3);  // wheel origin in IMU frame, read directly


    PRINT_DEBUG("[WHEEL] R_ItoO:\n[%.3f %.3f %.3f]\n[%.3f %.3f %.3f]\n[%.3f %.3f %.3f]\n",
                R_ItoO(0,0), R_ItoO(0,1), R_ItoO(0,2),
                R_ItoO(1,0), R_ItoO(1,1), R_ItoO(1,2),
                R_ItoO(2,0), R_ItoO(2,1), R_ItoO(2,2));
    // -----------------------------------------------------------------------
    // STEP 1: Residual with CURRENT estimates
    //
    //   Expected relative odometry (O0 frame):
    //     R_O0toO1_est = R_ItoO * R_GtoI1 * R_GtoI0^T * R_ItoO^T
    //     p_O1inO0_est = R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1^T*p_OinI
    //                                                  - p_I0inG - R_GtoI0^T*p_OinI)
    //
    //   Residuals (preintegrated - estimated), matching MINS sign convention:
    //     res_rot = -log(delta_R * R_O0toO1_est^T)
    //     res_pos =  delta_p - p_O1inO0_est
    // -----------------------------------------------------------------------

    Matrix3d R_GtoI0 = clone0->Rot();
    Vector3d p_I0inG = clone0->pos();
    Matrix3d R_GtoI1 = clone1->Rot();
    Vector3d p_I1inG = clone1->pos();

    Matrix3d R_O0toO1_est = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    Vector3d p_O1inO0_est = R_ItoO * R_GtoI0 *
                            (p_I1inG + R_GtoI1.transpose() * p_OinI
                                     - p_I0inG - R_GtoI0.transpose() * p_OinI);

    res = VectorXd::Zero(6);
    res.segment<3>(0) = -log_so3(delta_R * R_O0toO1_est.transpose());
    res.segment<3>(3) =  delta_p - p_O1inO0_est;

    PRINT_DEBUG("[WHEEL] Residual: rot=[%.4f,%.4f,%.4f] pos=[%.4f,%.4f,%.4f]\n",
                res(0),res(1),res(2),res(3),res(4),res(5));

    // -----------------------------------------------------------------------
    // STEP 2: Overwrite with FEJ values for Jacobian computation
    //
    //   FEJ (First Estimates Jacobians) is critical for filter consistency.
    //   Without it, the EKF becomes inconsistent because the Jacobians are
    //   evaluated at different linearization points in each update step,
    //   violating the observability structure of the system.
    // -----------------------------------------------------------------------

    R_GtoI0 = clone0->Rot_fej();
    p_I0inG = clone0->pos_fej();
    R_GtoI1 = clone1->Rot_fej();
    p_I1inG = clone1->pos_fej();

    // Recompute derived quantities with FEJ values
    Matrix3d R_I0toI1 = R_GtoI1 * R_GtoI0.transpose();
    // (R_O0toO1 not needed directly below but kept for reference)
    // Matrix3d R_O0toO1_fej = R_ItoO * R_I0toI1 * R_ItoO.transpose();

    // -----------------------------------------------------------------------
    // STEP 3: Compute Jacobians — mirrors MINS compute_linear_system_3D
    //
    //   Notation: dz_dx = d(predicted_z)/dx
    //   H = -dz_dx  (because res = z_meas - z_pred, so H = d(res)/dx = -dz_dx)
    //
    //   For rotation residual res_r = -log(delta_R * R_O0toO1^T):
    //     dzr/dth0 = -R_ItoO * R_GtoI1 * R_GtoI0^T  = -R_ItoO * R_I0toI1  (same as MINS)
    //     dzr/dth1 =  R_ItoO
    //
    //   For position residual res_p = delta_p - p_O1inO0:
    //     dzp/dth0 =  R_ItoO * skew(R_GtoI0*(p_I1inG - p_I0inG) + R_I0toI1^T*p_OinI_in_I0)
    //     dzp/dp0  = -R_ItoO * R_GtoI0
    //     dzp/dth1 = -R_ItoO * R_GtoI0 * R_GtoI1^T * skew(p_OinI)  = -R_ItoO * R_I0toI1^T * skew(p_OinI)
    //     dzp/dp1  =  R_ItoO * R_GtoI0
    //
    //   H = -dz_dx, so signs flip.
    //   BUT res = delta - estimated, so H = d(res)/dx = -d(estimated)/dx = -dz_dx
    //   We fill H = dz_dx directly (positive), matching MINS which also fills positive.
    // -----------------------------------------------------------------------

    x_order.clear();
    x_order.push_back(clone0);
    x_order.push_back(clone1);

    int total_hx = 0;
    for (const auto& var : x_order) total_hx += var->size();
    H = MatrixXd::Zero(6, total_hx);

    // Orientation Jacobians (directly match MINS dzr_dth0, dzr_dth1)
    Matrix3d dzr_dth0 = -R_ItoO * R_I0toI1;      // d(rot_residual)/d(theta_I0)
    Matrix3d dzr_dth1 =  R_ItoO;                  // d(rot_residual)/d(theta_I1)

    // Position Jacobians (directly match MINS dzp_dth0, dzp_dp0, dzp_dth1, dzp_dp1)
    // Note: p_I1inI0 = R_GtoI0 * (p_I1inG - p_I0inG)
    Vector3d p_I1inI0 = R_GtoI0 * (p_I1inG - p_I0inG);
    Matrix3d dzp_dth0 =  R_ItoO * skew_x(p_I1inI0 + R_I0toI1.transpose() * p_OinI);
    Matrix3d dzp_dp0  = -R_ItoO * R_GtoI0;
    Matrix3d dzp_dth1 = -R_ItoO * R_I0toI1.transpose() * skew_x(p_OinI);
    Matrix3d dzp_dp1  =  R_ItoO * R_GtoI0;

    // --- Clone 0 block (columns 0 .. clone0->size()-1) ---
    // PoseJPL layout: [delta_theta(3), delta_p(3), ...] — first 6 columns are orientation then position
    H.block<3,3>(0, 0) = dzr_dth0;
    // H(0, 3:5) = 0  (rotation residual independent of p0)
    H.block<3,3>(3, 0) = dzp_dth0;
    H.block<3,3>(3, 3) = dzp_dp0;

    // --- Clone 1 block (columns clone0->size() .. total_hx-1) ---
    int idx1 = clone0->size();
    H.block<3,3>(0, idx1+0) = dzr_dth1;
    // H(0, idx1+3 : idx1+5) = 0
    H.block<3,3>(3, idx1+0) = dzp_dth1;
    H.block<3,3>(3, idx1+3) = dzp_dp1;
    // x_order oluşturduktan ve H doldurduktan SONRA:
    MatrixXd P_marg = StateHelper::get_marginal_covariance(state, x_order);
    PRINT_DEBUG("[WHEEL] P_marg diagonal: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f | %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
        P_marg(0,0), P_marg(1,1), P_marg(2,2), P_marg(3,3), P_marg(4,4), P_marg(5,5),
        P_marg(6,6), P_marg(7,7), P_marg(8,8), P_marg(9,9), P_marg(10,10), P_marg(11,11));
    PRINT_DEBUG("[WHEEL] H row0: [%.4f %.4f %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f %.4f %.4f]\n",
        H(0,0),H(0,1),H(0,2),H(0,3),H(0,4),H(0,5),
        H(0,6),H(0,7),H(0,8),H(0,9),H(0,10),H(0,11));
    PRINT_DEBUG("[WHEEL] H row3: [%.4f %.4f %.4f %.4f %.4f %.4f | %.4f %.4f %.4f %.4f %.4f %.4f]\n",
        H(3,0),H(3,1),H(3,2),H(3,3),H(3,4),H(3,5),
        H(3,6),H(3,7),H(3,8),H(3,9),H(3,10),H(3,11));
    MatrixXd HPHt = H * P_marg * H.transpose();
    PRINT_DEBUG("[WHEEL] HPHt diagonal: [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n",
        HPHt(0,0),HPHt(1,1),HPHt(2,2),HPHt(3,3),HPHt(4,4),HPHt(5,5));

    PRINT_DEBUG("[WHEEL] P_marg full:\n");
    for(int r=0; r<12; r++){
        PRINT_DEBUG("[WHEEL] row%d: [%.6f %.6f %.6f %.6f %.6f %.6f | %.6f %.6f %.6f %.6f %.6f %.6f]\n",
            r,
            P_marg(r,0),P_marg(r,1),P_marg(r,2),P_marg(r,3),P_marg(r,4),P_marg(r,5),
            P_marg(r,6),P_marg(r,7),P_marg(r,8),P_marg(r,9),P_marg(r,10),P_marg(r,11));
    }
    PRINT_DEBUG("[WHEEL] H full:\n");
    for(int r=0; r<6; r++){
        PRINT_DEBUG("[WHEEL] Hrow%d: [%.6f %.6f %.6f %.6f %.6f %.6f | %.6f %.6f %.6f %.6f %.6f %.6f]\n",
            r,
            H(r,0),H(r,1),H(r,2),H(r,3),H(r,4),H(r,5),
            H(r,6),H(r,7),H(r,8),H(r,9),H(r,10),H(r,11));
    }
    // Sanity checks
    assert((int)H.rows() == 6);
    assert((int)H.cols() == total_hx);
    assert((int)res.rows() == 6);
    assert((int)covariance.rows() == 6 && (int)covariance.cols() == 6);

    PRINT_DEBUG("[WHEEL] H norm=%.4f, cov_trace=%.6f\n", H.norm(), covariance.trace());

    return true;
}


OdometryData UpdaterWheel::interpolate_data(const OdometryData& data1,
                                             const OdometryData& data2,
                                             double timestamp) {
    double lambda = (timestamp - data1.timestamp) / (data2.timestamp - data1.timestamp);
    lambda = std::max(0.0, std::min(1.0, lambda));

    OdometryData interp;
    interp.timestamp = timestamp;
    interp.linear_velocity  = (1.0 - lambda) * data1.linear_velocity  + lambda * data2.linear_velocity;
    interp.angular_velocity = (1.0 - lambda) * data1.angular_velocity + lambda * data2.angular_velocity;
    return interp;
}


// ============================================================
// SO(3) / Quaternion Utilities
// ============================================================

Matrix3d UpdaterWheel::exp_so3(const Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-8)
        return Matrix3d::Identity() + skew_x(omega);
    Matrix3d W = skew_x(omega);
    return Matrix3d::Identity()
           + (sin(theta) / theta) * W
           + ((1.0 - cos(theta)) / (theta * theta)) * W * W;
}

Vector3d UpdaterWheel::log_so3(const Matrix3d& R) {
    double cosval = std::min(1.0, std::max(-1.0, 0.5 * (R.trace() - 1.0)));
    double theta  = acos(cosval);
    if (theta < 1e-8)
        return Vector3d(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)) / 2.0;
    return theta / (2.0 * sin(theta)) *
           Vector3d(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
}

Matrix3d UpdaterWheel::skew_x(const Vector3d& v) {
    Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1),  v(0),  0;
    return S;
}

Matrix3d UpdaterWheel::Jr_so3(const Vector3d& w) {
    double theta = w.norm();
    if (theta < 1e-8)
        return Matrix3d::Identity() - 0.5 * skew_x(w);
    Matrix3d W = skew_x(w);
    return Matrix3d::Identity()
           - ((1.0 - cos(theta))  / (theta*theta))         * W
           + ((theta - sin(theta)) / (theta*theta*theta))   * W * W;
}

Matrix3d UpdaterWheel::Jr_so3_inv(const Vector3d& w) {
    double theta = w.norm();
    if (theta < 1e-8)
        return Matrix3d::Identity() + 0.5 * skew_x(w);
    Matrix3d W = skew_x(w);
    return Matrix3d::Identity()
           + 0.5 * W
           + (1.0/(theta*theta) - (1.0 + cos(theta)) / (2.0*theta*sin(theta))) * W * W;
}

Vector4d UpdaterWheel::rot_2_quat(const Matrix3d& R) {
    Vector4d q;
    double T = R.trace();
    if (T > 0) {
        double S = sqrt(T + 1.0) * 2.0;
        q << (R(2,1)-R(1,2))/S, (R(0,2)-R(2,0))/S, (R(1,0)-R(0,1))/S, 0.25*S;
    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
        double S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2.0;
        q << 0.25*S, (R(0,1)+R(1,0))/S, (R(0,2)+R(2,0))/S, (R(2,1)-R(1,2))/S;
    } else if (R(1,1) > R(2,2)) {
        double S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2.0;
        q << (R(0,1)+R(1,0))/S, 0.25*S, (R(1,2)+R(2,1))/S, (R(0,2)-R(2,0))/S;
    } else {
        double S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2.0;
        q << (R(0,2)+R(2,0))/S, (R(1,2)+R(2,1))/S, 0.25*S, (R(1,0)-R(0,1))/S;
    }
    return q / q.norm();
}

Matrix3d UpdaterWheel::quat_2_Rot(const Vector4d& q) {
    double qx=q(0), qy=q(1), qz=q(2), qw=q(3);
    Matrix3d R;
    R << 1-2*qy*qy-2*qz*qz,  2*qx*qy-2*qz*qw,  2*qx*qz+2*qy*qw,
         2*qx*qy+2*qz*qw,  1-2*qx*qx-2*qz*qz,  2*qy*qz-2*qx*qw,
         2*qx*qz-2*qy*qw,    2*qy*qz+2*qx*qw,  1-2*qx*qx-2*qy*qy;
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
    Matrix4d Om;
    Om <<  0,     w(2), -w(1),  w(0),
          -w(2),  0,     w(0),  w(1),
           w(1), -w(0),  0,     w(2),
          -w(0), -w(1), -w(2),  0;
    return Om;
}
