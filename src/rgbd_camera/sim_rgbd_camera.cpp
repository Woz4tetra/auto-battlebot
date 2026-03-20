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

bool SimRgbdCamera::get(CameraData &data, [[maybe_unused]] bool get_depth) {
    if (!connection_->is_connected()) return false;
    if (!connection_->step_and_receive(data)) return false;

    if (!data.ground_truth_poses.empty()) {
        DiagnosticsData gt_data;
        const auto &gt = data.ground_truth_poses;
        gt_data["our_x"] = gt[0].x;
        gt_data["our_y"] = gt[0].y;
        gt_data["our_yaw_deg"] = gt[0].yaw * 180.0 / M_PI;
        for (size_t i = 1; i < gt.size(); ++i) {
            std::string prefix = "opp" + std::to_string(i) + "_";
            gt_data[prefix + "x"] = gt[i].x;
            gt_data[prefix + "y"] = gt[i].y;
            gt_data[prefix + "yaw_deg"] = gt[i].yaw * 180.0 / M_PI;
        }
        logger_->debug("ground_truth", gt_data);
    }

    return true;
}

bool SimRgbdCamera::should_close() { return !connection_->is_connected(); }

}  // namespace auto_battlebot
