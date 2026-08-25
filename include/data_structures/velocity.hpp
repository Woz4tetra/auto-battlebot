#pragma once

namespace auto_battlebot {
/** Planar velocity in the field frame. Shared by robot tracks and by the hazards derived from
 * them, so it lives here rather than in robot.hpp (field.hpp cannot include that: robot.hpp
 * includes field.hpp). */
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
