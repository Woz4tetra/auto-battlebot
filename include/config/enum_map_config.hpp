#pragma once

#include <vector>
#include <map>
#include <string>
#include <magic_enum.hpp>
#include "config/config_parser.hpp"
#include "enums/label.hpp"
#include "enums/keypoint_label.hpp"

namespace auto_battlebot
{
    /**
     * Configuration for mapping Labels to their corresponding KeypointLabels
     * Parses TOML format (array of tables for guaranteed order):
     * [[section]]
     * label = "LABEL_NAME"
     * keypoints = ["KEYPOINT_LABEL_1", "KEYPOINT_LABEL_2"]
     * 
     * Uses ordered storage to ensure indices match model class order
     */
    struct LabelToKeypointMapConfiguration
    {
        std::vector<std::pair<Label, std::vector<KeypointLabel>>> label_to_keypoints;

        void parse(ConfigParser &parser, const std::string &field_name)
        {
            const toml::array *array_ptr = parser.get_array(field_name);
            if (!array_ptr)
            {
                throw ConfigValidationError("Missing required field '" + field_name + "' (expected array of tables)");
            }

            for (const auto &item : *array_ptr)
            {
                const toml::table *entry = item.as_table();
                if (!entry)
                {
                    throw ConfigValidationError("Each entry in '" + field_name + "' must be a table");
                }

                // Get label
                auto label_node = entry->get("label");
                if (!label_node)
                {
                    throw ConfigValidationError("Missing 'label' field in " + field_name + " entry");
                }
                auto label_str = label_node->value<std::string>();
                if (!label_str)
                {
                    throw ConfigValidationError("'label' must be a string in " + field_name + " entry");
                }

                auto label_opt = magic_enum::enum_cast<Label>(*label_str);
                if (!label_opt.has_value())
                {
                    throw ConfigValidationError("Invalid Label: " + *label_str);
                }
                Label label = label_opt.value();

                // Get keypoints array
                auto keypoints_node = entry->get("keypoints");
                if (!keypoints_node)
                {
                    throw ConfigValidationError("Missing 'keypoints' field for label " + *label_str);
                }
                auto keypoint_array = keypoints_node->as_array();
                if (!keypoint_array)
                {
                    throw ConfigValidationError("'keypoints' must be an array for label " + *label_str);
                }

                std::vector<KeypointLabel> keypoint_labels;
                for (const auto &kp_toml : *keypoint_array)
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

                label_to_keypoints.emplace_back(label, keypoint_labels);
            }
        }

        // Helper to get keypoint labels for a given object label
        const std::vector<KeypointLabel> &get_keypoint_labels(Label label) const
        {
            static const std::vector<KeypointLabel> empty;
            for (const auto &[l, kps] : label_to_keypoints)
            {
                if (l == label)
                {
                    return kps;
                }
            }
            return empty;
        }

        // Get label at a specific index (for model class id lookup)
        Label get_label_at_index(size_t index) const
        {
            if (index < label_to_keypoints.size())
            {
                return label_to_keypoints[index].first;
            }
            return Label::EMPTY;
        }

        size_t size() const
        {
            return label_to_keypoints.size();
        }
    };

} // namespace auto_battlebot
