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
