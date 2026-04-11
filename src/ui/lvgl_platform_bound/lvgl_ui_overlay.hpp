#pragma once

#include <opencv2/core.hpp>
#include <optional>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "navigation/navigation_interface.hpp"

namespace auto_battlebot::ui_internal {

void draw_robot_pose_arrows(cv::Mat &image, const RobotDescriptionsStamped &robots,
                            const FieldDescription &field, const CameraInfo &camera_info);
void draw_field_border(cv::Mat &image, const FieldDescription &field,
                       const CameraInfo &camera_info);
void draw_target_path_overlay(cv::Mat &image, const std::optional<NavigationPathSegment> &path,
                              const FieldDescription &field, const CameraInfo &camera_info);

}  // namespace auto_battlebot::ui_internal
