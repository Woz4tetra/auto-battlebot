#pragma once

#include "data_structures.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "config_factory.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    struct KeypointModelConfiguration
    {
        std::string type;
        virtual ~KeypointModelConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopKeypointModelConfiguration : public KeypointModelConfiguration
    {
        NoopKeypointModelConfiguration()
        {
            type = "NoopKeypointModel";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config);
    std::unique_ptr<KeypointModelConfiguration> parse_keypoint_model_config(ConfigParser &parser);
} // namespace auto_battlebot
