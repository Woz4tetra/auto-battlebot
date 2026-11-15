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
    Eigen::Matrix4d tf_camera_from_fieldcenter = Eigen::Matrix4d::Identity();
    double residual_px = 0.0;
};

std::array<cv::Point2d, 4> field_object_corners(double size_x, double size_y);

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

/**
 * @brief Turn the field frame about its own z so +x runs left to right in the camera view.
 *
 * A mat outline fits equally well at every quarter turn of a square, or every half turn of a
 * rectangle, so the corner order alone leaves +x wherever the image winding put it. This picks
 * the candidate closest to the camera's x axis tipped onto the field plane, the same choice
 * PointCloudFieldFilter makes. Translation is unchanged: every candidate shares the centre.
 */
Eigen::Matrix4d align_field_x_with_camera_x(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                            double size_x, double size_y);

FieldPoseResult pose_from_outline(const FieldOutline &outline, double size_x, double size_y,
                                  const Eigen::Matrix3d &intrinsics);

std::array<cv::Point2d, 4> project_field_corners(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                                 double size_x, double size_y,
                                                 const Eigen::Matrix3d &intrinsics);

bool intrinsics_to_eigen(const cv::Mat &intrinsics, Eigen::Matrix3d &out);
}  // namespace auto_battlebot
