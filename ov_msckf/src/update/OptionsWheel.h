#ifndef OV_MSCKF_OPTIONS_WHEEL_H
#define OV_MSCKF_OPTIONS_WHEEL_H

#include <Eigen/Eigen>
#include <string>

namespace ov_msckf {

struct OptionsWheel {

    std::string topic = "/wheel_odom";

    double noise_v = 0.1; // Lineer hız gürültüsü
    double noise_w = 0.1; // Açısal hız gürültüsü

    double chi2_mult = 1.0;

    Eigen::Matrix4d T_imu_wheel = Eigen::Matrix4d::Identity();
};

} // namespace OV_MSCKF

#endif // OV_MSCKF_OPTIONS_WHEEL_H

