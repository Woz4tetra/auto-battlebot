#include "rgbd_camera/sim_rgbd_camera.hpp"

#include <spdlog/spdlog.h>

namespace auto_battlebot {

SimRgbdCamera::SimRgbdCamera(const SimRgbdCameraConfiguration &config) {
    SimConnection::configure(config.sim_host, config.sim_port);
    connection_ = SimConnection::instance();
}

bool SimRgbdCamera::initialize() { return connection_->connect(); }

bool SimRgbdCamera::get(CameraData &data, [[maybe_unused]] bool get_depth) {
    if (!connection_->is_connected()) return false;
    return connection_->step_and_receive(data);
}

bool SimRgbdCamera::should_close() { return !connection_->is_connected(); }

}  // namespace auto_battlebot
