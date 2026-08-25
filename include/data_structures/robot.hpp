#pragma once

#include <string>
#include <vector>

#include "enums.hpp"
#include "field.hpp"
#include "header.hpp"
#include "pose.hpp"
#include "velocity.hpp"

namespace auto_battlebot {
struct RobotDescription {
    FrameId frame_id = FrameId::EMPTY;
    Label label = Label::EMPTY;
    Group group = Group::OURS;
    Pose pose;
    Size size;
    std::vector<Position> keypoints = std::vector<Position>();
    Velocity2D velocity;
    /** Footprint of the robot. For labels listed in [robot_filter.robot_size_meters_per_label]
     * this is the known dimensions, not the detector's box; the box that was measured is logged
     * to the `detected_size` diagnostics channel instead of being carried here. */
    bool is_stale = false;
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
