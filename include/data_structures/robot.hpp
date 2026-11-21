#pragma once

#include <optional>
#include <string>
#include <vector>

#include "enums.hpp"
#include "field.hpp"
#include "header.hpp"
#include "pose.hpp"
#include "velocity.hpp"

namespace auto_battlebot {
struct PoseUncertainty {
    std::optional<double> position_sigma_major_axis_m = std::nullopt;
    std::optional<double> heading_sigma_rad = std::nullopt;
    std::optional<double> yaw_rate_sigma_rad_s = std::nullopt;
};

struct RobotDescription {
    FrameId frame_id = FrameId::EMPTY;
    Label label = Label::EMPTY;
    Group group = Group::OURS;
    Pose pose;
    Size size;
    std::vector<Position> keypoints = std::vector<Position>();
    Velocity2D velocity;
    PoseUncertainty uncertainty;
    Position2D footprint_center_offset{};
};

struct RobotDescriptionsStamped {
    Header header;
    std::vector<RobotDescription> descriptions;
};

struct RobotConfig {
    Label label = Label::MR_STABS_MK1;
    Group group = Group::OURS;
};

}  // namespace auto_battlebot
