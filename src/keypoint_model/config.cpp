#include "keypoint_model/config.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(KeypointModelConfiguration, NoopKeypointModelConfiguration, "NoopKeypointModel")

    std::unique_ptr<KeypointModelConfiguration> parse_keypoint_model_config(ConfigParser &parser)
    {
        return ConfigFactory<KeypointModelConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config)
    {
        if (config.type == "NoopKeypointModel")
        {
            return std::make_shared<NoopKeypointModel>();
        }
        throw std::invalid_argument("Failed to load KeypointModel of type " + config.type);
    }
} // namespace auto_battlebot
