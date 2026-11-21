#include "command_shaper/identity_command_shaper.hpp"

namespace auto_battlebot {

IdentityCommandShaper::IdentityCommandShaper(double velocity_saturation_limit)
    : velocity_saturation_limit_(velocity_saturation_limit) {}

StickCommand IdentityCommandShaper::shape(const VelocityCommand &command) const {
    return saturate_stick({.forward = command.forward_mps,
                           .lateral = command.lateral_mps,
                           .yaw = command.yaw_rate_radps},
                          velocity_saturation_limit_);
}

}  // namespace auto_battlebot
