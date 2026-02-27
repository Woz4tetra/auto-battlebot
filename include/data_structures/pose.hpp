#pragma once

namespace auto_battlebot {
struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Rotation {
    double w = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Pose {
    Position position;
    Rotation rotation;
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

}  // namespace auto_battlebot
