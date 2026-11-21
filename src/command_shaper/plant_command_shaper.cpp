#include "command_shaper/plant_command_shaper.hpp"

namespace auto_battlebot {

StickCommand plant_body_velocity_to_stick(const VelocityCommand &command,
                                          const JigPlantParams &plant) {
    const double linear_gain = command.forward_mps >= 0.0 ? plant.k_fwd : plant.k_rev;
    return {.forward = command.forward_mps / linear_gain,
            .lateral = 0.0,
            .yaw = command.yaw_rate_radps / plant.k_ang};
}

PlantCommandShaper::PlantCommandShaper(const JigPlantParams &plant,
                                       double velocity_saturation_limit)
    : plant_(plant), velocity_saturation_limit_(velocity_saturation_limit) {}

StickCommand PlantCommandShaper::shape(const VelocityCommand &command) const {
    return saturate_stick(plant_body_velocity_to_stick(command, plant_),
                          velocity_saturation_limit_);
}

}  // namespace auto_battlebot
