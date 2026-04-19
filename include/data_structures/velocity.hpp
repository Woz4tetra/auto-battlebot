#pragma once

namespace auto_battlebot {
struct VelocityCommand {
    double linear_x;   // forward velocity in m/s
    double linear_y;   // lateral velocity in m/s
    double angular_z;  // yaw rate in rad/s
};

}  // namespace auto_battlebot
