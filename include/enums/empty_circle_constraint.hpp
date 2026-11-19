#pragma once

namespace auto_battlebot {
enum class EmptyCircleConstraint {
    NONE,
    FIELD_CENTER,
    THREE_WALL,
    OPPOSITE_WALLS_POINT,
    OPPOSITE_WALLS_HAZARD,
    PERPENDICULAR_WALLS_POINT,
    PERPENDICULAR_WALLS_HAZARD,
    WALL_POINT_PAIR,
    WALL_SITE_PAIR,
    POINT_TRIPLE,
    SITE_TRIPLE
};
}  // namespace auto_battlebot
