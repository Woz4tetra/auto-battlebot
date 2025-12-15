#pragma once

#include "data_structures.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"

namespace auto_battlebot
{
    struct KeypointModelConfiguration
    {
        std::string type;
    };

    struct NoopKeypointModelConfiguration : public KeypointModelConfiguration
    {
        NoopKeypointModelConfiguration()
        {
            type = "NoopKeypointModel";
        }
    };

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config);
} // namespace auto_battlebot
