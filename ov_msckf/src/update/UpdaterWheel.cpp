#include "UpdaterWheel.h"
#include "utils/print.h"
#include "state/StateHelper.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace Eigen;


UpdaterWheel::UpdaterWheel(std::shared_ptr<State> state) : state(state) {

}


void UpdaterWheel::feed_measurement(const OdometryData& message, double oldest_time) {
    std::lock_guard<std::mutex> lck(wheel_data_mtx);
    wheel_data.push_back(message);

    clean_old_measurements(oldest_time);
}


void UpdaterWheel::try_update() {}


bool UpdaterWheel::update(double time0, double time1) {}


bool UpdaterWheel::select_wheel_data(double time0, double time1, std::vector<OdometryData>& data_vec) {}

void ov_msckf::UpdaterWheel::clean_old_measurements(double oldest_time) {
// Negatif zaman gelirse temizlik yapma (başlangıç durumu)
    if (oldest_time < 0) return;

    std::lock_guard<std::mutex> lck(wheel_data_mtx); // DİKKAT: Temizlerken de kilitlemelisin!
    
    auto it0 = wheel_data.begin();
    while (it0 != wheel_data.end()) {
        // Eğer verinin zamanı, silinmesi gereken zamandan eskiyse sil
        if (it0->timestamp < oldest_time) {
            it0 = wheel_data.erase(it0); // Vektörden siler ve iteratörü günceller
        } else {
            // Zaman sıralı olduğu için, ilk "yeni" veriyi gördüğün an döngüden çıkabilirsin.
            // Bu performans artışı sağlar (OpenVINS kodunda bu 'break' yok ama ekleyebilirsin).
            // break; 
            it0++;
        }
    }
}
