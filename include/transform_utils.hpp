#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

#include "data_structures/camera.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/transform.hpp"

namespace auto_battlebot {
/**
 * @brief Convert a 4x4 transformation matrix to position and quaternion
 * @param transform 4x4 transformation matrix (homogeneous coordinates)
 * @param position Output position (x, y, z)
 * @param quaternion Output quaternion (w, x, y, z)
 */
void matrix_to_position_quaternion(const Eigen::Matrix4d &transform, Position &position,
                                   Rotation &quaternion);

/**
 * @brief Convert a 4x4 transformation matrix to position and Euler angles
 * @param transform 4x4 transformation matrix (homogeneous coordinates)
 * @param position Output position (x, y, z)
 * @param roll Output roll angle (rotation around x-axis)
 * @param pitch Output pitch angle (rotation around y-axis)
 * @param yaw Output yaw angle (rotation around z-axis)
 */
void matrix_to_position_euler(const Eigen::Matrix4d &transform, Position &position, double &roll,
                              double &pitch, double &yaw);

/**
 * @brief Convert position and quaternion to a 4x4 transformation matrix
 * @param position Position (x, y, z)
 * @param quaternion Quaternion (w, x, y, z)
 * @return 4x4 transformation matrix
 */
Eigen::Matrix4d position_quaternion_to_matrix(const Position &position, const Rotation &quaternion);

/**
 * @brief Convert position and Euler angles to a 4x4 transformation matrix
 * @param position Position (x, y, z)
 * @param roll Roll angle (rotation around x-axis)
 * @param pitch Pitch angle (rotation around y-axis)
 * @param yaw Yaw angle (rotation around z-axis)
 * @return 4x4 transformation matrix
 */
Eigen::Matrix4d position_euler_to_matrix(const Position &position, double roll, double pitch,
                                         double yaw);

/**
 * @brief Convert Pose (position + quaternion) to 4x4 transformation matrix
 * @param pose Pose containing position and rotation (quaternion)
 * @return 4x4 transformation matrix
 */
Eigen::Matrix4d pose_to_matrix(const Pose &pose);

/**
 * @brief Convert 4x4 transformation matrix to Pose (position + quaternion)
 * @param transform 4x4 transformation matrix
 * @return Pose containing position and rotation (quaternion)
 */
Pose matrix_to_pose(const Eigen::Matrix4d &transform);

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param quaternion Quaternion (w, x, y, z)
 * @param roll Output roll angle
 * @param pitch Output pitch angle
 * @param yaw Output yaw angle
 */
void quaternion_to_euler(const Rotation &quaternion, double &roll, double &pitch, double &yaw);

/**
 * @brief Convert Euler angles to quaternion
 * @param roll Roll angle
 * @param pitch Pitch angle
 * @param yaw Yaw angle
 * @return Quaternion (w, x, y, z)
 */
Rotation euler_to_quaternion(double roll, double pitch, double yaw);

/**
 * @brief Invert a TransformStamped (swap parent and child frames)
 * @param transform The transform to invert
 * @return Inverted transform with swapped parent/child frames
 */
TransformStamped invert_transform(const TransformStamped &transform);

/**
 * @brief Convert a TransformStamped to a human-readable string representation
 * @param transform The transform to convert
 * @return String representation with frame IDs, position, and rotation
 */
std::string transform_to_string(const TransformStamped &transform);

/**
 * @brief Convert an Eigen Vector3d to a Position struct
 * @param vector The vector to convert
 * @return Position struct
 */
Position vector_to_position(const Eigen::Vector3d &vector);

/**
 * @brief Convert a 3D Pose (position + quaternion) to a 2D pose (x, y, yaw)
 * @param pose 3D Pose
 * @return Pose2D with x, y from position and yaw extracted from quaternion
 */
Pose2D pose_to_pose2d(const Pose &pose);

/**
 * @brief Convert a 2D pose to a 3D Pose (z=0, roll=0, pitch=0)
 * @param pose2d 2D pose
 * @return Pose with position (x, y, 0) and quaternion from (0, 0, yaw)
 */
Pose pose2d_to_pose(const Pose2D &pose2d);

/**
 * @brief Convert a 2D pose to a Position (z=0)
 * @param pose2d 2D pose
 * @return Position with x, y from pose2d and z=0
 */
Position pose2d_to_position(const Pose2D &pose2d);

/**
 * @brief Build a normalized camera-frame ray from a pixel and camera intrinsics.
 * @param camera_info Camera intrinsics
 * @param pixel_x Pixel x coordinate in image space
 * @param pixel_y Pixel y coordinate in image space
 * @param out_ray Output ray direction in camera frame
 * @return True when intrinsics are valid and ray is finite
 */
bool pixel_to_camera_ray(const CameraInfo &camera_info, double pixel_x, double pixel_y,
                         Eigen::Vector3d &out_ray);

/**
 * @brief Intersect a camera-frame ray (origin at camera) with a plane in camera frame.
 * @param ray Camera-frame ray direction
 * @param plane_center Any point on the plane in camera frame
 * @param plane_normal Plane normal in camera frame
 * @param out_point Intersection point in camera frame
 * @return True if intersection exists in front of camera
 */
bool intersect_camera_ray_with_plane(const Eigen::Vector3d &ray,
                                     const Eigen::Vector3d &plane_center,
                                     const Eigen::Vector3d &plane_normal,
                                     Eigen::Vector3d &out_point);

/**
 * @brief Build a plane (center + normal) from a field-to-camera transform.
 * The plane corresponds to the transformed field z=0 plane expressed in camera frame.
 * @param transform Transform containing camera_from_field matrix
 * @param out_center A point on the plane in camera frame
 * @param out_normal Unit normal in camera frame
 * @return True when transform is valid and plane can be constructed
 */
bool transform_to_plane_center_normal(const Transform &transform, Eigen::Vector3d &out_center,
                                      Eigen::Vector3d &out_normal);

/**
 * @brief Project a keypoint pixel to a 3D point on a known plane in camera frame.
 * @param keypoint Input keypoint with pixel coordinates
 * @param plane_center Any point on the plane in camera frame
 * @param plane_normal Plane normal in camera frame
 * @param camera_info Camera intrinsics
 * @param out_point Projected point on plane in camera frame
 * @return True when projection succeeds
 */
bool project_keypoint_onto_plane(const Keypoint &keypoint, const Eigen::Vector3d &plane_center,
                                 const Eigen::Vector3d &plane_normal, const CameraInfo &camera_info,
                                 Eigen::Vector3d &out_point);

/**
 * @brief Transform a 3D point by a homogeneous 4x4 transform.
 */
Eigen::Vector3d transform_point(const Eigen::Matrix4d &tf, const Eigen::Vector3d &point);

/**
 * @brief Rotate body-frame linear velocity into field frame.
 * @return XY velocity in field frame
 */
Eigen::Vector2d body_velocity_to_field(double linear_x, double linear_y, double yaw);

}  // namespace auto_battlebot
