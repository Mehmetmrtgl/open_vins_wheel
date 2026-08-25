#ifndef OV_MSCKF_OPTIONS_WHEEL_H
#define OV_MSCKF_OPTIONS_WHEEL_H

#include <Eigen/Eigen>
#include <string>

namespace ov_msckf {

struct OptionsWheel {

    std::string topic = "/wheel_odom";

    // ---- Ackermann shorthand ----
    // Three sigmas covering the three roles an ackermann odometry message has:
    // the encoder-derived forward velocity, the wheel-derived yaw rate, and the
    // four channels the message reports as zero.
    double noise_v = 0.1; // Forward velocity noise — linear.x (m/s)
    double noise_w = 0.1; // Yaw rate noise — angular.z (rad/s)
    double noise_p = 0.1; // Near-zero axes: angular.x/y, linear.y/z

    // ---- Per-axis body-frame odometry noise ----
    // What the preintegration actually consumes. Seeded from the three sigmas
    // above by apply_scalar_defaults() so an ackermann config needs nothing
    // else, and overridable outright for a platform whose channels do not fall
    // into those three roles. Order: [x, y, z] in the odometry body frame.
    //
    // These sigmas are where the vehicle's kinematics enter the wheel channel.
    // A car reports v = [v_x, 0, 0]; the zeros on y and z are not dead
    // channels, they are the nonholonomic fact asserted at every sample, and
    // the sigma states how far the true body velocity may depart from zero
    // (tyre slip, suspension travel). Inflating them on the premise that those
    // channels carry no signal throws that information away: measured on
    // urban39 as 8.915 m -> 12.951 m APE.
    Eigen::Vector3d noise_w_axis = Eigen::Vector3d(0.1, 0.1, 0.1);
    Eigen::Vector3d noise_v_axis = Eigen::Vector3d(0.1, 0.1, 0.1);

    double chi2_mult = 1.0;

    /// Skip wheel update when angular velocity exceeds threshold (configurable)
    bool do_turn_detection = false;
    double turn_ang_threshold = 0.3;  ///< rad/s

    Eigen::Matrix4d T_imu_wheel = Eigen::Matrix4d::Identity();

    /// Lay the ackermann shorthand out over the six body axes. Called after the
    /// scalars are parsed and before the optional per-axis keys, so an explicit
    /// noise_v_axis / noise_w_axis in yaml still wins.
    void apply_scalar_defaults() {
        noise_w_axis = Eigen::Vector3d(noise_p, noise_p, noise_w);  // ang x, y, z(yaw rate)
        noise_v_axis = Eigen::Vector3d(noise_v, noise_p, noise_p);  // lin x(forward), y, z
    }
};

} // namespace ov_msckf

#endif // OV_MSCKF_OPTIONS_WHEEL_H
