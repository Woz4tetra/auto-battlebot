#pragma once

namespace auto_battlebot
{
    struct Position
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Rotation
    {
        double w = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Pose
    {
        Position position;
        Rotation rotation;
    };

} // namespace auto_battlebot
