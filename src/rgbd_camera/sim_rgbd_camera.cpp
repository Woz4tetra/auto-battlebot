#include "rgbd_camera/sim_rgbd_camera.hpp"

#include <spdlog/spdlog.h>

#include <cmath>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {

SimRgbdCamera::SimRgbdCamera(const SimRgbdCameraConfiguration &config)
    : logger_(DiagnosticsLogger::get_logger("sim_camera")) {
    SimConnection::configure(config.sim_host, config.sim_port);
    connection_ = SimConnection::instance();
}

bool SimRgbdCamera::initialize() { return connection_->connect(); }

bool SimRgbdCamera::get(CameraData &data) {
    if (!connection_->is_connected()) return false;
    if (!connection_->step_and_receive(data)) return false;

    if (data.ground_truth) {
        DiagnosticsData gt_data;
        const auto &gt = *data.ground_truth;
        gt_data["our_x"] = gt.our_robot.x;
        gt_data["our_y"] = gt.our_robot.y;
        gt_data["our_yaw_deg"] = gt.our_robot.yaw * 180.0 / M_PI;
        for (size_t i = 0; i < gt.opponents.size(); ++i) {
            std::string prefix = "opp" + std::to_string(i + 1) + "_";
            gt_data[prefix + "x"] = gt.opponents[i].x;
            gt_data[prefix + "y"] = gt.opponents[i].y;
            gt_data[prefix + "yaw_deg"] = gt.opponents[i].yaw * 180.0 / M_PI;
        }
        logger_->debug("ground_truth", gt_data);
    }

    return true;
}

bool SimRgbdCamera::should_close() { return !connection_->is_connected(); }

}  // namespace auto_battlebot
