#pragma once

#include <optional>
#include <vector>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "enums/empty_circle_constraint.hpp"

namespace auto_battlebot {

struct EmptyCircle {
    Pose2D center{};
    double radius = 0.0;
    int evaluations = 0;
    EmptyCircleConstraint constraint = EmptyCircleConstraint::NONE;
};

double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents,
                           const std::vector<FieldHazard> &hazards);

double empty_circle_radius(const Size &field, double x, double y,
                           const std::vector<Pose2D> &opponents);

EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::vector<FieldHazard> &hazards,
                        const std::optional<Pose2D> &tie_break_near = std::nullopt);

EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents,
                        const std::optional<Pose2D> &tie_break_near = std::nullopt);

}  // namespace auto_battlebot
