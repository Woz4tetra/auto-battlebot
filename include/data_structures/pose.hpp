#pragma once

namespace auto_battlebot {

struct Position {
    double x;
    double y;
    double z;
};

struct Rotation {
    double w;
    double x;
    double y;
    double z;
};

struct Pose {
    Position position;
    Rotation rotation;
};

}  // namespace auto_battlebot
