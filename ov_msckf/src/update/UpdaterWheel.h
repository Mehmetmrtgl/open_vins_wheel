#ifndef OV_MSCKF_UPDATER_WHEEL_H
#define OV_MSCKF_UPDATER_WHEEL_H

#include "state/State.h"
#include "utils/sensor_data.h"
#include "OptionsWheel.h"
#include <deque>

namespace ov_msckf {

class UpdaterWheel {
public:
    // Constructor
    UpdaterWheel(std::shared_ptr<State> state);

    // Veri besleme fonksiyonu (ROS Callback'ten buraya gelecek)
    void feed_measurement(const ov_core::OdometryData& message, double oldest_time);

    // Ana güncelleme tetikleyicisi (VioManager çağıracak)
    void try_update();

    std::mutex odometry_data_mtx;          // 1. Kilit mekanizması
    std::vector<ov_core::OdometryData> odometry_data; // 2. Veri deposu

private:
    // Belirli iki zaman arasındaki güncellemeyi yapan iç fonksiyon
    bool update(double time0, double time1);

    // Zaman aralığındaki verileri seçen yardımcı fonksiyon
    bool select_odometry_data(double time0, double time1, std::vector<ov_core::OdometryData>& data_vec);

    void clean_old_measurements(double oldest_time);

    void preintegration_3D(double dt,
                        const ov_core::OdometryData& data1,
                        const ov_core::OdometryData& data2);

    // Durum (State) pointer'ı
    std::shared_ptr<State> state;

    // Son güncelleme zamanı
    double last_updated_clone_time = -1.0;

    // --- Ön Entegrasyon Değişkenleri ---
    Eigen::Vector3d delta_p;  // Birikmiş pozisyon değişimi
    Eigen::Matrix3d delta_R;  // Birikmiş rotasyon değişimi
    Eigen::Matrix<double, 6, 6> covariance; // Birikmiş belirsizlik
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_WHEEL_H