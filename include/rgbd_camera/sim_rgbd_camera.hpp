#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "rgbd_camera/config.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "simulation/sim_connection.hpp"

namespace auto_battlebot {

class SimRgbdCamera : public RgbdCameraInterface {
   public:
    explicit SimRgbdCamera(const SimRgbdCameraConfiguration &config);

    bool initialize() override;
    bool get(CameraData &data, bool get_depth) override;
    bool should_close() override;

   private:
    std::shared_ptr<SimConnection> connection_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
