#include "rgbd_camera/config.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"
#include "rgbd_camera/zed_rgbd_camera.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(RgbdCameraConfiguration, NoopRgbdCameraConfiguration, "NoopRgbdCamera")
    REGISTER_CONFIG(RgbdCameraConfiguration, ZedRgbdCameraConfiguration, "ZedRgbdCamera")

    std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser)
    {
        return ConfigFactory<RgbdCameraConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for RgbdCamera" << std::endl;
        if (config.type == "NoopRgbdCamera")
        {
            return std::make_shared<NoopRgbdCamera>();
        }
        else if (config.type == "ZedRgbdCamera")
        {
            return std::make_shared<ZedRgbdCamera>(
                config_cast<ZedRgbdCameraConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load RgbdCamera of type " + config.type);
    }
} // namespace auto_battlebot
