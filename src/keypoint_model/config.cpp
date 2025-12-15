#include "keypoint_model/config.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"

namespace auto_battlebot
{
    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config)
    {
        if (config.type == "NoopKeypointModel")
        {
            return std::make_shared<NoopKeypointModel>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
