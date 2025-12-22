#include "transform_utils.hpp"

namespace auto_battlebot
{
    void matrix_to_position_quaternion(
        const Eigen::Matrix4d &transform,
        Position &position,
        Rotation &quaternion)
    {
        // Extract position
        position.x = transform(0, 3);
        position.y = transform(1, 3);
        position.z = transform(2, 3);

        // Extract rotation matrix and convert to quaternion
        Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation_matrix);
        quat.normalize();

        quaternion.w = quat.w();
        quaternion.x = quat.x();
        quaternion.y = quat.y();
        quaternion.z = quat.z();
    }

    void matrix_to_position_euler(
        const Eigen::Matrix4d &transform,
        Position &position,
        double &roll,
        double &pitch,
        double &yaw)
    {
        // Extract position
        position.x = transform(0, 3);
        position.y = transform(1, 3);
        position.z = transform(2, 3);

        // Extract rotation matrix and convert to Euler angles (ZYX convention)
        Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order

        yaw = euler_angles(0);
        pitch = euler_angles(1);
        roll = euler_angles(2);
    }

    Eigen::Matrix4d position_quaternion_to_matrix(
        const Position &position,
        const Rotation &quaternion)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        // Create quaternion and convert to rotation matrix
        Eigen::Quaterniond quat(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        quat.normalize();
        transform.block<3, 3>(0, 0) = quat.toRotationMatrix();

        // Set translation
        transform(0, 3) = position.x;
        transform(1, 3) = position.y;
        transform(2, 3) = position.z;

        return transform;
    }

    Eigen::Matrix4d position_euler_to_matrix(
        const Position &position,
        double roll,
        double pitch,
        double yaw)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        // Create rotation matrix from Euler angles (ZYX convention: yaw-pitch-roll)
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        // Combined rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
        transform.block<3, 3>(0, 0) = rotation;

        // Set translation
        transform(0, 3) = position.x;
        transform(1, 3) = position.y;
        transform(2, 3) = position.z;

        return transform;
    }

    Eigen::Matrix4d pose_to_matrix(const Pose &pose)
    {
        return position_quaternion_to_matrix(pose.position, pose.rotation);
    }

    Pose matrix_to_pose(const Eigen::Matrix4d &transform)
    {
        Pose pose;
        matrix_to_position_quaternion(transform, pose.position, pose.rotation);
        return pose;
    }

    void quaternion_to_euler(
        const Rotation &quaternion,
        double &roll,
        double &pitch,
        double &yaw)
    {
        // Convert quaternion to rotation matrix, then to Euler angles
        Eigen::Quaterniond quat(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        quat.normalize();
        
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order

        yaw = euler_angles(0);
        pitch = euler_angles(1);
        roll = euler_angles(2);
    }

    Rotation euler_to_quaternion(double roll, double pitch, double yaw)
    {
        // Create rotation matrix from Euler angles (ZYX convention)
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        // Combined rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
        quat.normalize();

        Rotation rotation;
        rotation.w = quat.w();
        rotation.x = quat.x();
        rotation.y = quat.y();
        rotation.z = quat.z();

        return rotation;
    }

} // namespace auto_battlebot
