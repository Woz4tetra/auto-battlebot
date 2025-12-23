#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include "data_structures/transform.hpp"
#include "data_structures/pose.hpp"

namespace auto_battlebot
{
    /**
     * @brief Convert a 4x4 transformation matrix to position and quaternion
     * @param transform 4x4 transformation matrix (homogeneous coordinates)
     * @param position Output position (x, y, z)
     * @param quaternion Output quaternion (w, x, y, z)
     */
    void matrix_to_position_quaternion(
        const Eigen::Matrix4d &transform,
        Position &position,
        Rotation &quaternion);

    /**
     * @brief Convert a 4x4 transformation matrix to position and Euler angles
     * @param transform 4x4 transformation matrix (homogeneous coordinates)
     * @param position Output position (x, y, z)
     * @param roll Output roll angle (rotation around x-axis)
     * @param pitch Output pitch angle (rotation around y-axis)
     * @param yaw Output yaw angle (rotation around z-axis)
     */
    void matrix_to_position_euler(
        const Eigen::Matrix4d &transform,
        Position &position,
        double &roll,
        double &pitch,
        double &yaw);

    /**
     * @brief Convert position and quaternion to a 4x4 transformation matrix
     * @param position Position (x, y, z)
     * @param quaternion Quaternion (w, x, y, z)
     * @return 4x4 transformation matrix
     */
    Eigen::Matrix4d position_quaternion_to_matrix(
        const Position &position,
        const Rotation &quaternion);

    /**
     * @brief Convert position and Euler angles to a 4x4 transformation matrix
     * @param position Position (x, y, z)
     * @param roll Roll angle (rotation around x-axis)
     * @param pitch Pitch angle (rotation around y-axis)
     * @param yaw Yaw angle (rotation around z-axis)
     * @return 4x4 transformation matrix
     */
    Eigen::Matrix4d position_euler_to_matrix(
        const Position &position,
        double roll,
        double pitch,
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
    void quaternion_to_euler(
        const Rotation &quaternion,
        double &roll,
        double &pitch,
        double &yaw);

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

} // namespace auto_battlebot
