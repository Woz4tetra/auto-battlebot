#pragma once

#include "keypoint_model/keypoint_model_interface.hpp"

namespace auto_battlebot
{
    class NoopKeypointModel : public KeypointModelInterface
    {
    public:
        bool initialize() override { return true; }

        KeypointsStamped update([[maybe_unused]] RgbImage image) override
        {
            return KeypointsStamped{};
        }
    };

} // namespace auto_battlebot
