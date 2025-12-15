#include "rgbd_camera/config.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"

namespace auto_battlebot
{
    std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config)
    {
        if (config.type == "NoopRgbdCamera")
        {
            return std::make_shared<NoopRgbdCamera>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
