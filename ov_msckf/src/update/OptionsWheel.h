#ifndef OV_MSCKF_OPTIONS_WHEEL_H
#define OV_MSCKF_OPTIONS_WHEEL_H

#include <Eigen/Eigen>
#include <string>

namespace ov_msckf {

struct OptionsWheel {

    std::string topic = "/wheel_odom";

    double noise_v = 0.1; // Forward velocity noise — linear.x (m/s)
    double noise_w = 0.1; // Yaw rate noise — angular.z (rad/s)
    double noise_p = 0.1; // Near-zero axes: angular.x/y, linear.y/z (ackermann constraint)

    double chi2_mult = 1.0;

    /// Skip wheel update when angular velocity exceeds threshold (configurable)
    bool do_turn_detection = false;
    double turn_ang_threshold = 0.3;  ///< rad/s

    Eigen::Matrix4d T_imu_wheel = Eigen::Matrix4d::Identity();
};

} // namespace OV_MSCKF

#endif // OV_MSCKF_OPTIONS_WHEEL_H

