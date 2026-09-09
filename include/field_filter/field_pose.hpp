#pragma once

#include <Eigen/Dense>
#include <array>
#include <opencv2/opencv.hpp>
#include <string>

#include "field_filter/field_outline.hpp"

namespace auto_battlebot {
struct FieldPoseResult {
    bool ok = false;
    std::string failure;
    /** Field centre expressed in camera coordinates. */
    Eigen::Matrix4d tf_camera_from_fieldcenter = Eigen::Matrix4d::Identity();
    /** Mean reprojection residual, in pixels. Both solves are exactly determined, so this is a
     *  numerical sanity check and not a quality guard: the four-corner homography fits whatever
     *  corners it is handed at 0.0 px however wrong they are, and the three-line solve puts six
     *  constraints against six degrees of freedom. The structural guards in FieldOutline are
     *  what catch a bad fit here. FiducialFieldFilter is the one place a residual measures
     *  something, because 15 markers give 60 correspondences against the same six unknowns. */
    double residual_px = 0.0;
};

/** Object corners in the field frame, wound to match FieldOutline::corners. */
std::array<cv::Point2d, 4> field_object_corners(double size_x, double size_y);

/** Pose from four image corners and the known metric field size.
 *
 * H = K [r1 r2 t] up to scale, so K^-1 H recovers two rotation columns and the translation and
 * the third column is their cross product. The result is orthonormalized, because measured
 * corners never satisfy the constraint exactly. */
FieldPoseResult pose_from_corners(const std::array<cv::Point2d, 4> &image_corners, double size_x,
                                  double size_y, const Eigen::Matrix3d &intrinsics);

/**
 * @brief Pose from three field edges, when the fourth runs off the frame.
 *
 * At a cage mount the near mat corners fall outside the frame, so there is no four-corner fit to
 * make. Three sides are still enough: the two opposite ones meet at the vanishing point of their
 * shared world direction, which fixes one rotation column outright; the third side's backprojected
 * plane is perpendicular to the second column, which fixes that; and the two finite corners, a
 * known distance apart, fix the translation and the scale. Six constraints for six degrees of
 * freedom, with the missing corners falling out as intersections outside the image.
 *
 * `side_supported` must have exactly three entries true. `observed_corners` is read only for its
 * winding: a rectangle mirrored about one of its own axes is the same rectangle, so a mirrored
 * field frame satisfies all three lines exactly and only the winding separates it from the right
 * answer.
 */
FieldPoseResult pose_from_three_lines(const std::array<cv::Vec3d, 4> &lines,
                                      const std::array<bool, 4> &side_supported,
                                      const std::array<cv::Point2d, 4> &observed_corners,
                                      double size_x, double size_y,
                                      const Eigen::Matrix3d &intrinsics);

/** Whichever of the two solves the outline supports. */
FieldPoseResult pose_from_outline(const FieldOutline &outline, double size_x, double size_y,
                                  const Eigen::Matrix3d &intrinsics);

/** Project the field corners back into the image through a recovered pose. Used for the overlay
 *  and for the seating check, and it is the only way to get corners for a clipped outline. */
std::array<cv::Point2d, 4> project_field_corners(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                                 double size_x, double size_y,
                                                 const Eigen::Matrix3d &intrinsics);

/** Intrinsics as Eigen, from the CameraInfo matrix. Returns false when the matrix is not 3x3. */
bool intrinsics_to_eigen(const cv::Mat &intrinsics, Eigen::Matrix3d &out);
}  // namespace auto_battlebot
