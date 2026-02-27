#include "rgbd_camera/config.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"
#include "rgbd_camera/zed_rgbd_camera.hpp"
#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "config/config_parser.hpp"
#include <toml++/toml.h>

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(RgbdCameraConfiguration, NoopRgbdCameraConfiguration, "NoopRgbdCamera")
    REGISTER_CONFIG(RgbdCameraConfiguration, ZedRgbdCameraConfiguration, "ZedRgbdCamera")
    REGISTER_CONFIG(RgbdCameraConfiguration, SimRgbdCameraConfiguration, "SimRgbdCamera")

    std::unique_ptr<RgbdCameraConfiguration> parse_rgbd_camera_config(ConfigParser &parser)
    {
        return ConfigFactory<RgbdCameraConfiguration>::instance().create_and_parse(parser);
    }

    std::unique_ptr<RgbdCameraConfiguration> load_camera_from_toml(
        toml::table const &toml_data,
        std::vector<std::string> &parsed_sections)
    {
        auto section = toml_data["rgbd_camera"].as_table();
        if (!section)
        {
            throw ConfigValidationError("Missing required section [rgbd_camera]");
        }
        ConfigParser parser(*section, "rgbd_camera");
        auto config = parse_rgbd_camera_config(parser);
        parsed_sections.push_back("rgbd_camera");
        return config;
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
        else if (config.type == "SimRgbdCamera")
        {
            return std::make_shared<SimRgbdCamera>(
                config_cast<SimRgbdCameraConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load RgbdCamera of type " + config.type);
    }
} // namespace auto_battlebot
