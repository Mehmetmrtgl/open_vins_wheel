#include "UpdaterWheel.h"
#include "utils/print.h"
#include "state/StateHelper.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace Eigen;


UpdaterWheel::UpdaterWheel(std::shared_ptr<State> state) : state(state) {

}


void UpdaterWheel::feed_measurement(const OdometryData& message) {}


void UpdaterWheel::try_update() {}


bool UpdaterWheel::update(double time0, double time1) {}


bool UpdaterWheel::select_wheel_data(double time0, double time1, std::vector<OdometryData>& data_vec) {}