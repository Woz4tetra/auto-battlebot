#pragma once

#include "command_shaper/command_shaper_interface.hpp"
#include "plant/plant_params.hpp"

namespace auto_battlebot {

class PlantCommandShaper : public CommandShaperInterface {
   public:
    PlantCommandShaper(const JigPlantParams &plant, double velocity_saturation_limit);

    StickCommand shape(const VelocityCommand &command) const override;

   private:
    JigPlantParams plant_;
    double velocity_saturation_limit_;
};

StickCommand plant_body_velocity_to_stick(const VelocityCommand &command,
                                          const JigPlantParams &plant);

}  // namespace auto_battlebot
