#pragma once

namespace auto_battlebot {
struct Velocity2D {
    double vx_mps = 0.0;
    double vy_mps = 0.0;
    double omega_radps = 0.0;
};

struct StickCommand {
    double forward = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
};

struct VelocityCommand {
    double forward_mps;
    double lateral_mps;
    double yaw_rate_radps;
};

}  // namespace auto_battlebot
