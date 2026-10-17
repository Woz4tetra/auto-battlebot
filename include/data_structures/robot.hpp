#pragma once

#include <limits>
#include <string>
#include <vector>

#include "enums.hpp"
#include "field.hpp"
#include "header.hpp"
#include "pose.hpp"
#include "velocity.hpp"

namespace auto_battlebot {
/**
 * One-sigma uncertainty of a rendered robot estimate. Infinity means the estimator has no
 * information on that quantity: a held or dead-reckoned track, an opponent whose heading has never
 * been measured, or a label listed as heading-blind.
 */
struct PoseUncertainty {
    /** Square root of the larger eigenvalue of the 2x2 position covariance. */
    double position_sigma_m = std::numeric_limits<double>::infinity();
    double heading_sigma_rad = std::numeric_limits<double>::infinity();
    double yaw_rate_sigma_rad_s = std::numeric_limits<double>::infinity();
};

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
    PoseUncertainty uncertainty;
    /** Seconds from the track's last accepted measurement to the time this estimate was rendered.
     * 0 on a fresh frame. Filters that do not track it leave 0. */
    double measurement_age_s = 0.0;
    /** Centre of the robot's footprint in its body frame (x forward, y left), in metres from
     *  `pose`. `pose` is the point control tracks, our robot's axle; hazard and clearance checks
     *  measure from the footprint centre instead, through footprint_center(). Set per label from
     *  [robot_filter.footprint_center_offset_per_label]; zero where the pose is the centre. */
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
