#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "enums/hazard_kind.hpp"
#include "enums/label.hpp"
#include "header.hpp"
#include "point_cloud.hpp"
#include "pose.hpp"
#include "transform.hpp"
#include "velocity.hpp"

namespace auto_battlebot {
struct Mask {
    Label label;
    cv::Mat mask;
};

struct MaskStamped {
    Header header;
    Mask mask;
};

struct Size {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct SizeStamped {
    Header header;
    Size size;
};

enum class HazardSource { STATIC, TRACKED };

/** A region to keep out of, in the field frame. Discs regardless of the real shape: every
 * consumer reduces to a distance-to-centre test, and EmptyCircleSolver is already built on point
 * sites plus a radius. Cover an awkward shape with two or three overlapping discs rather than
 * teaching the consumers a second geometry.
 *
 * `radius` is already inflated. Inflation happens once, at assembly, never at a consumer: the
 * stored radius is the raw hazard radius plus our robot's half-diagonal plus the configured
 * margin, so every consumer can treat it as "centre distance below this is a loss" without
 * re-deriving clearance. */
struct FieldHazard {
    Pose2D center;
    double inflated_radius_m = 0.0;
    double object_radius = 0.0;
    double hard_radius = 0.0;
    Velocity2D velocity;
    HazardSource source = HazardSource::STATIC;
    HazardKind kind = HazardKind::HOLE;
};

struct FieldDescription {
    Header header;
    FrameId child_frame_id = FrameId::EMPTY;
    Transform tf_camera_from_fieldcenter;
    SizeStamped size;
    std::vector<FieldHazard> hazards;
};

struct FieldDescriptionWithInlierPoints : FieldDescription {
    PointCloud inlier_points;
};

}  // namespace auto_battlebot
