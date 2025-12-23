#pragma once

#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot
{
    class NoopRgbdCamera : public RgbdCameraInterface
    {
    public:
        bool initialize() override { return true; }
        bool get(CameraData &data) const override
        {
            data = camera_data_;
            return true;
        }
        bool should_close() override { return false; }

    private:
        CameraData camera_data_;
    };

} // namespace auto_battlebot
