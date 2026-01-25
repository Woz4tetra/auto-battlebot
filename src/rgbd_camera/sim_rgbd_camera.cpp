#include "rgbd_camera/sim_rgbd_camera.hpp"

namespace auto_battlebot
{
    SimRgbdCamera::SimRgbdCamera(SimRgbdCameraConfiguration &config) : expected_width_(config.width),
                                                                       expected_height_(config.height)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_rgbd_camera");
    }

    SimRgbdCamera::~SimRgbdCamera()
    {
    }

    bool SimRgbdCamera::initialize()
    {
    }

    bool SimRgbdCamera::get(CameraData &data, bool get_depth) const
    {
    }

    bool SimRgbdCamera::should_close()
    {
    }
} // namespace auto_battlebot
