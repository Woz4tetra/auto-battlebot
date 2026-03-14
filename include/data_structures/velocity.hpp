#pragma once

namespace auto_battlebot {
struct VelocityCommand {
    double linear_x;   // nomalized command -1..+1
    double linear_y;   // nomalized command -1..+1
    double angular_z;  // nomalized command -1..+1
};

}  // namespace auto_battlebot
