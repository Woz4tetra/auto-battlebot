#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RobotBlobModelInterface {
   public:
    virtual ~RobotBlobModelInterface() = default;
    virtual bool initialize() = 0;
    virtual KeypointsStamped update(RgbImage image) = 0;
};
}  // namespace auto_battlebot

