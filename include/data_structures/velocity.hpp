#pragma once

namespace auto_battlebot {
struct Velocity2D {
    double vx = 0.0;     // m/s in field frame x
    double vy = 0.0;     // m/s in field frame y
    double omega = 0.0;  // rad/s heading rate
};

struct VelocityCommand {
    double linear_x;   // forward velocity in m/s
    double linear_y;   // lateral velocity in m/s
    double angular_z;  // yaw rate in rad/s
};

}  // namespace auto_battlebot
