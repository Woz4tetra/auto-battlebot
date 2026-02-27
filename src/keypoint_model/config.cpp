#include "keypoint_model/config.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"
#include "keypoint_model/yolo_keypoint_model.hpp"
#include "config/config_parser.hpp"
#include "config/config_cast.hpp"
#include <toml++/toml.h>

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(KeypointModelConfiguration, NoopKeypointModelConfiguration, "NoopKeypointModel")
    REGISTER_CONFIG(KeypointModelConfiguration, YoloKeypointModelConfiguration, "YoloKeypointModel")

    std::unique_ptr<KeypointModelConfiguration> parse_keypoint_model_config(ConfigParser &parser)
    {
        return ConfigFactory<KeypointModelConfiguration>::instance().create_and_parse(parser);
    }

    std::unique_ptr<KeypointModelConfiguration> load_keypoint_model_from_toml(
        toml::table const &toml_data,
        std::vector<std::string> &parsed_sections)
    {
        auto section = toml_data["keypoint_model"].as_table();
        if (!section)
        {
            throw ConfigValidationError("Missing required section [keypoint_model]");
        }
        ConfigParser parser(*section, "keypoint_model");
        auto config = parse_keypoint_model_config(parser);
        parsed_sections.push_back("keypoint_model");
        return config;
    }

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for KeypointModel" << std::endl;
        if (config.type == "NoopKeypointModel")
        {
            return std::make_shared<NoopKeypointModel>();
        }
        else if (config.type == "YoloKeypointModel")
        {
            return std::make_shared<YoloKeypointModel>(
                config_cast<YoloKeypointModelConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load KeypointModel of type " + config.type);
    }
} // namespace auto_battlebot
