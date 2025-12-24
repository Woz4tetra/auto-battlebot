#pragma once

#include "data_structures.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "config/config_factory.hpp"
#include "config/config_cast.hpp"
#include "config/config_parser.hpp"
#include "config/enum_map_config.hpp"
#include "enums/label.hpp"
#include "enums/keypoint_label.hpp"

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

    struct YoloKeypointModelConfiguration : public KeypointModelConfiguration
    {
        std::string model_path;
        float threshold = 0.50f;
        float iou_threshold = 0.45f;
        float letterbox_padding = 0.1f;
        int image_size = 640;
        LabelToKeypointMapConfiguration label_map;
        bool debug_visualization = false;

        YoloKeypointModelConfiguration()
        {
            type = "YoloKeypointModel";
        }

        void parse_fields(ConfigParser &parser) override
        {
            model_path = parser.get_required_string("model_path");
            threshold = parser.get_optional_double("threshold", threshold);
            iou_threshold = parser.get_optional_double("iou_threshold", iou_threshold);
            letterbox_padding = parser.get_optional_double("letterbox_padding", letterbox_padding);
            image_size = parser.get_optional_int("image_size", image_size);
            label_map.parse(parser, "label_map");
            debug_visualization = parser.get_optional_bool("debug_visualization", debug_visualization);
            parser.validate_no_extra_fields();
        }
    };

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config);
    std::unique_ptr<KeypointModelConfiguration> parse_keypoint_model_config(ConfigParser &parser);
} // namespace auto_battlebot
