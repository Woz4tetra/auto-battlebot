#pragma once

#include "command_shaper/command_shaper_interface.hpp"

namespace auto_battlebot {

class IdentityCommandShaper : public CommandShaperInterface {
   public:
    explicit IdentityCommandShaper(double velocity_saturation_limit);

    StickCommand shape(const VelocityCommand &command) const override;

   private:
    double velocity_saturation_limit_;
};

}  // namespace auto_battlebot
