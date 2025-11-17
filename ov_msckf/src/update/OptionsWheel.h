#ifndef OV_CORE_OPTIONS_WHEEL_H
#define OV_CORE_OPTIONS_WHEEL_H

#include <Eigen/Eigen>
#include <string>

namespace ov_core {

struct OptionsWheel {
    bool enabled = false;
    std::string topic = "/wheel_odom";
    
    // Gürültü Parametreleri (Noise)
    double noise_v = 0.1; // Lineer hız gürültüsü
    double noise_w = 0.1; // Açısal hız gürültüsü
    
    // Kalibrasyon Seçenekleri
    bool do_calib_ext = false; // Extrinsic (T_imu_wheel) kalibre edilsin mi?
    
    // Extrinsic: Wheel'den IMU'ya dönüşüm (YAML'da T_imu_wheel)
    Eigen::Matrix4d T_imu_wheel = Eigen::Matrix4d::Identity(); 
    
    // Chi-Square Testi eşiği
    double chi2_mult = 1.0;
};

} // namespace ov_core

#endif // OV_CORE_OPTIONS_WHEEL_H