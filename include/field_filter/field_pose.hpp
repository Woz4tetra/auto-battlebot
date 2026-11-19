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

FieldPoseResult pose_from_three_lines(const std::array<cv::Vec3d, 4> &lines,
                                      const std::array<bool, 4> &side_supported,
                                      const std::array<cv::Point2d, 4> &observed_corners,
                                      double size_x, double size_y,
                                      const Eigen::Matrix3d &intrinsics);

Eigen::Matrix4d align_field_x_with_camera_x(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                            double size_x, double size_y);

FieldPoseResult pose_from_outline(const FieldOutline &outline, double size_x, double size_y,
                                  const Eigen::Matrix3d &intrinsics);

std::array<cv::Point2d, 4> project_field_corners(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                                                 double size_x, double size_y,
                                                 const Eigen::Matrix3d &intrinsics);

bool intrinsics_to_eigen(const cv::Mat &intrinsics, Eigen::Matrix3d &out);
}  // namespace auto_battlebot
