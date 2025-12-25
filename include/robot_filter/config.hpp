#pragma once

#include "data_structures.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot
{
    struct RobotFilterConfiguration
    {
        std::string type;
        virtual ~RobotFilterConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopRobotFilterConfiguration : public RobotFilterConfiguration
    {
        NoopRobotFilterConfiguration()
        {
            type = "NoopRobotFilter";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    struct RobotFrontBackKalmanFilterConfiguration : public RobotFilterConfiguration
    {
        std::vector<KeypointLabel> front_keypoints;
        std::vector<KeypointLabel> back_keypoints;
        std::map<Label, std::vector<FrameId>> label_to_frame_ids;

        RobotFrontBackKalmanFilterConfiguration()
        {
            type = "RobotFrontBackKalmanFilter";
        }

        void parse_fields(ConfigParser &parser) override
        {
            front_keypoints = parse_keypoints(parser, "front_keypoints");
            back_keypoints = parse_keypoints(parser, "back_keypoints");
            parse_label_to_frame_ids(parser, "label_mapping");
            parser.validate_no_extra_fields();
        }

        void parse_label_to_frame_ids(ConfigParser &parser, const std::string &field_name)
        {
            const toml::table *table_ptr = parser.get_table(field_name);
            if (!table_ptr)
            {
                throw ConfigValidationError("Missing required field '" + field_name + "'");
            }

            const toml::table &table = *table_ptr;

            for (const auto &[key, value] : table)
            {
                std::string label_str(key.str());

                // Parse the label
                auto label_opt = magic_enum::enum_cast<Label>(label_str);
                if (!label_opt.has_value())
                {
                    throw ConfigValidationError("Invalid Label: " + label_str);
                }
                Label label = label_opt.value();

                // Parse the array of frame IDs
                auto frame_id_array = value.as_array();
                if (!frame_id_array)
                {
                    throw ConfigValidationError("Value for '" + label_str + "' must be an array");
                }

                std::vector<FrameId> frame_ids;
                for (const auto &fr_toml : *frame_id_array)
                {
                    auto fr_str = fr_toml.template value<std::string>();
                    if (fr_str)
                    {
                        auto fr_opt = magic_enum::enum_cast<FrameId>(*fr_str);
                        if (fr_opt.has_value())
                        {
                            frame_ids.push_back(fr_opt.value());
                        }
                        else
                        {
                            throw ConfigValidationError("Invalid KeypointLabel: " + *fr_str);
                        }
                    }
                }

                label_to_frame_ids[label] = frame_ids;
            }
        }

        std::vector<KeypointLabel> parse_keypoints(ConfigParser &parser, const std::string &field_name)
        {
            const toml::array *array_ptr = parser.get_array(field_name);
            if (!array_ptr)
            {
                throw ConfigValidationError("Missing required field '" + field_name + "'");
            }

            if (!array_ptr)
            {
                throw ConfigValidationError("Value for front_keypoints must be an array");
            }
            auto keypoint_array_table = *array_ptr;
            std::vector<KeypointLabel> keypoint_labels;
            for (const auto &kp_toml : keypoint_array_table)
            {
                auto kp_str = kp_toml.template value<std::string>();
                if (kp_str)
                {
                    auto kp_opt = magic_enum::enum_cast<KeypointLabel>(*kp_str);
                    if (kp_opt.has_value())
                    {
                        keypoint_labels.push_back(kp_opt.value());
                    }
                    else
                    {
                        throw ConfigValidationError("Invalid KeypointLabel: " + *kp_str);
                    }
                }
            }
            return keypoint_labels;
        }
    };

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config);
    std::unique_ptr<RobotFilterConfiguration> parse_robot_filter_config(ConfigParser &parser);
} // namespace auto_battlebot
