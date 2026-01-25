#pragma once

#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "rgbd_camera/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{
    class SimRgbdCamera : public RgbdCameraInterface
    {
    public:
        SimRgbdCamera(SimRgbdCameraConfiguration &config);
        ~SimRgbdCamera();
        bool initialize() override;
        bool get(CameraData &data, bool get_depth) const override;
        bool should_close() override;

    private:
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        int expected_width_, expected_height_;
    };

} // namespace auto_battlebot
