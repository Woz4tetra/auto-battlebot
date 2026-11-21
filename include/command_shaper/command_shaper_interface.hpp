#pragma once

#include "data_structures/velocity.hpp"

namespace auto_battlebot {

class CommandShaperInterface {
   public:
    virtual ~CommandShaperInterface() = default;
    virtual StickCommand shape(const VelocityCommand &command) const = 0;
};

StickCommand saturate_stick(StickCommand stick, double saturation_limit);

}  // namespace auto_battlebot
