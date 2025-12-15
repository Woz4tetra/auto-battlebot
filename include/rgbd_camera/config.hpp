#pragma once

#include "data_structures.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"

namespace auto_battlebot
{
    struct RgbdCameraConfiguration
    {
        std::string type;
    };

    struct NoopRgbdCameraConfiguration : public RgbdCameraConfiguration
    {
        NoopRgbdCameraConfiguration()
        {
            type = "NoopRgbdCamera";
        }
    };

    std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config);
} // namespace auto_battlebot
