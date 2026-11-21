#include "command_shaper/plant_command_shaper.hpp"

#include <cmath>

namespace auto_battlebot {

StickCommand plant_body_velocity_to_stick(const VelocityCommand &command,
                                          const JigPlantParams &plant) {
    const double linear_gain = command.forward_mps >= 0.0 ? plant.k_fwd : plant.k_rev;
    return {.forward = command.forward_mps / linear_gain,
            .lateral = 0.0,
            .yaw = command.yaw_rate_radps / plant.k_ang};
}

double lift_past_deadzone(double stick, double deadzone) {
    if (deadzone > 0.0 && std::abs(stick) > 1e-6) {
        return std::copysign(std::abs(stick) * (1.0 - deadzone) + deadzone, stick);
    }
    return stick;
}

PlantCommandShaper::PlantCommandShaper(const JigPlantParams &plant,
                                       double velocity_saturation_limit, double linear_deadzone)
    : plant_(plant),
      velocity_saturation_limit_(velocity_saturation_limit),
      linear_deadzone_(linear_deadzone) {}

StickCommand PlantCommandShaper::shape(const VelocityCommand &command) const {
    StickCommand stick = plant_body_velocity_to_stick(command, plant_);
    stick.forward = lift_past_deadzone(stick.forward, linear_deadzone_);
    return saturate_stick(stick, velocity_saturation_limit_);
}

}  // namespace auto_battlebot
