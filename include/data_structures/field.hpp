#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

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

/** Where a hazard came from. Decides how it decays and how diagnostics read it: a STATIC hazard
 * is arena geometry known before the match and never disappears, a TRACKED one is a live robot
 * track and can go stale. */
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
    /** Keep-out radius: the geometry inflated by our robot's half-diagonal and the configured
     * margin. This is what navigation steers around, because it steers a point. */
    double inflated_radius = 0.0;
    /** The hazard itself, before inflation: the hole in the floor, or the robot's own footprint.
     *
     * This is what the overlays draw. Drawing `inflated_radius` instead read as the hazard being
     * twice its real size, because the inflation that makes it a configuration-space obstacle has
     * no visible cause; the robot circles on screen give the eye enough to judge the margin without
     * a second ring for every hazard. */
    double object_radius = 0.0;
    Velocity2D velocity;
    HazardSource source = HazardSource::STATIC;
};

struct FieldDescription {
    Header header;
    FrameId child_frame_id = FrameId::EMPTY;
    Transform tf_camera_from_fieldcenter;
    SizeStamped size;
    /** Keep-out discs for this cycle, assembled from config geometry and live neutral tracks.
     * Empty on a field with nothing to avoid. */
    std::vector<FieldHazard> hazards;
};

struct FieldDescriptionWithInlierPoints : FieldDescription {
    PointCloud inlier_points;
};

}  // namespace auto_battlebot
