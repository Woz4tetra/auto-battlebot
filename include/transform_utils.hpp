#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

#include "data_structures/camera.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/transform.hpp"

namespace auto_battlebot {
void matrix_to_position_quaternion(const Eigen::Matrix4d &transform, Position &position,
                                   Rotation &quaternion);

void matrix_to_position_euler(const Eigen::Matrix4d &transform, Position &position, double &roll,
                              double &pitch, double &yaw);

Eigen::Matrix4d position_quaternion_to_matrix(const Position &position, const Rotation &quaternion);

Eigen::Matrix4d position_euler_to_matrix(const Position &position, double roll, double pitch,
                                         double yaw);

Eigen::Matrix4d pose_to_matrix(const Pose &pose);

Pose matrix_to_pose(const Eigen::Matrix4d &transform);

void quaternion_to_euler(const Rotation &quaternion, double &roll, double &pitch, double &yaw);

Rotation euler_to_quaternion(double roll, double pitch, double yaw);

TransformStamped invert_transform(const TransformStamped &transform);

std::string transform_to_string(const TransformStamped &transform);

Position vector_to_position(const Eigen::Vector3d &vector);

Pose2D pose_to_pose2d(const Pose &pose);

Pose pose2d_to_pose(const Pose2D &pose2d);

Position pose2d_to_position(const Pose2D &pose2d);

bool pixel_to_camera_ray(const CameraInfo &camera_info, double pixel_x, double pixel_y,
                         Eigen::Vector3d &out_ray);

bool intersect_camera_ray_with_plane(const Eigen::Vector3d &ray,
                                     const Eigen::Vector3d &plane_center,
                                     const Eigen::Vector3d &plane_normal,
                                     Eigen::Vector3d &out_point);

bool transform_to_plane_center_normal(const Transform &transform, Eigen::Vector3d &out_center,
                                      Eigen::Vector3d &out_normal);

bool project_keypoint_onto_plane(const Keypoint &keypoint, const Eigen::Vector3d &plane_center,
                                 const Eigen::Vector3d &plane_normal, const CameraInfo &camera_info,
                                 Eigen::Vector3d &out_point);

/**
 * @brief Project a keypoint pixel onto a plane, then compensate for the physical keypoint
 *        sitting `keypoint_height` above the plane by shifting the result inward toward the
 *        camera nadir.
 * @param keypoint Input keypoint with pixel coordinates
 * @param plane_center Any point on the plane in camera frame
 * @param plane_normal Plane normal in camera frame
 * @param camera_info Camera intrinsics
 * @param keypoint_height Height of the physical keypoint above the plane in meters
 * @param out_point Corrected on-plane point in camera frame
 * @return True when projection succeeds
 */
bool project_keypoint_onto_plane(const Keypoint &keypoint, const Eigen::Vector3d &plane_center,
                                 const Eigen::Vector3d &plane_normal, const CameraInfo &camera_info,
                                 double keypoint_height, Eigen::Vector3d &out_point);

/**
 * @brief Shift an on-plane projected point inward toward the camera nadir to compensate for the
 *        keypoint sitting `keypoint_height` above the plane. Returns the input unchanged when the
 *        geometry is degenerate (camera at or below keypoint height).
 * @param plane_point On-plane point from ray/plane intersection, in camera frame
 * @param plane_center Any point on the plane in camera frame
 * @param plane_normal Unit plane normal in camera frame
 * @param keypoint_height Height of the physical keypoint above the plane in meters
 * @return Corrected on-plane point in camera frame
 */
Eigen::Vector3d correct_plane_height_offset(const Eigen::Vector3d &plane_point,
                                            const Eigen::Vector3d &plane_center,
                                            const Eigen::Vector3d &plane_normal,
                                            double keypoint_height);

Eigen::Vector3d transform_point(const Eigen::Matrix4d &tf, const Eigen::Vector3d &point);

/**
 * @brief Rotate body-frame linear velocity into field frame.
 * @return XY velocity in field frame
 */
Eigen::Vector2d body_velocity_to_field(double linear_x, double linear_y, double yaw);

}  // namespace auto_battlebot
