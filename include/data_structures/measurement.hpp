#pragma once

#include "data_structures/robot.hpp"

namespace auto_battlebot {

struct MeasurementWithConfidence {
    double confidence;
    RobotDescription description;
};

}  // namespace auto_battlebot
