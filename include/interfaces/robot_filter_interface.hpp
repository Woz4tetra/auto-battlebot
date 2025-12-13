#pragma once

#include "data_structures.hpp"
#include <vector>

namespace auto_battlebot {

class RobotFilterInterface {
public:
    virtual ~RobotFilterInterface() = default;
    virtual bool initialize(const std::vector<RobotConfig>& robots) = 0;
    virtual RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field) = 0;
};

}  // namespace auto_battlebot
