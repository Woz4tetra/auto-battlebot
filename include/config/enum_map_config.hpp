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
     * Parses TOML format: [section]
     * "LABEL_NAME" = ["KEYPOINT_LABEL_1", "KEYPOINT_LABEL_2"]
     */
    struct LabelToKeypointMapConfiguration
    {
        std::map<Label, std::vector<KeypointLabel>> label_to_keypoints;

        void parse(ConfigParser &parser, const std::string &field_name)
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

                // Parse the array of keypoint labels
                auto keypoint_array = value.as_array();
                if (!keypoint_array)
                {
                    throw ConfigValidationError("Value for '" + label_str + "' must be an array");
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

                label_to_keypoints[label] = keypoint_labels;
            }
        }

        // Helper to get keypoint labels for a given object label
        const std::vector<KeypointLabel> &get_keypoint_labels(Label label) const
        {
            static const std::vector<KeypointLabel> empty;
            auto it = label_to_keypoints.find(label);
            if (it != label_to_keypoints.end())
            {
                return it->second;
            }
            return empty;
        }
    };

} // namespace auto_battlebot
