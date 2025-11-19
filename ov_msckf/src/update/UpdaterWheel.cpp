#include "UpdaterWheel.h"
#include "utils/print.h"
#include "state/StateHelper.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace Eigen;


UpdaterWheel::UpdaterWheel(std::shared_ptr<State> state) : state(state) {

}


void UpdaterWheel::feed_measurement(const OdometryData& message, double oldest_time) {
    std::lock_guard<std::mutex> lck(odometry_data_mtx);
    odometry_data.push_back(message);

    clean_old_measurements(oldest_time);
}


void UpdaterWheel::try_update() {}


bool UpdaterWheel::update(double time0, double time1) {
    // Şimdilik sadece iskelet: veri seç ve preintegration uygula.
    std::vector<OdometryData> data_vec;
    if (!select_odometry_data(time0, time1, data_vec) || data_vec.size() < 2) {
        return false;
    }

    // Preintegrasyonu sıfırla
    delta_p.setZero();
    delta_R.setIdentity();
    covariance.setZero();

    for (size_t i = 0; i + 1 < data_vec.size(); ++i) {
        const auto& d1 = data_vec[i];
        const auto& d2 = data_vec[i + 1];

        double dt = d2.timestamp - d1.timestamp;
        if (dt <= 0.0) continue;


        // preintegration_3D(dt, , ); 
    }

    last_updated_clone_time = time1;
    return true;
}

bool UpdaterWheel::select_odometry_data(double time0, double time1,
                                     std::vector<OdometryData>& data_vec) {

    std::lock_guard<std::mutex> lck(odometry_data_mtx);

    if (odometry_data.empty() || time1 <= time0) {
        return false;
    }

    for (const auto& msg : odometry_data) {
        if (msg.timestamp < time0) continue;
        if (msg.timestamp > time1) break;
        data_vec.push_back(msg);
    }

    return !data_vec.empty();
}

void ov_msckf::UpdaterWheel::clean_old_measurements(double oldest_time) {
// Negatif zaman gelirse temizlik yapma (başlangıç durumu)
    if (oldest_time < 0) return;
    
    auto it0 = odometry_data.begin();
    while (it0 != odometry_data.end()) {
        // Eğer verinin zamanı, silinmesi gereken zamandan eskiyse sil
        if (it0->timestamp < oldest_time) {
            it0 = odometry_data.erase(it0); // Vektörden siler ve iteratörü günceller
        } else {
            it0++;
        }
    }
}

void ov_msckf::UpdaterWheel::preintegration_3D(double dt, const ov_core::OdometryData &data1, const ov_core::OdometryData &data2) {


}
