#pragma once

#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot
{
    class NoopRgbdCamera : public RgbdCameraInterface
    {
    public:
        bool initialize() override { return true; }
        bool update() override { return true; }
        bool get([[maybe_unused]] CameraData &data) override { return false; }
        bool should_close() override { return false; }
    };

} // namespace auto_battlebot
