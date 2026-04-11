#include "transform_utils.hpp"

#include <iomanip>
#include <sstream>

#include "enum_to_string_lower.hpp"

namespace auto_battlebot {
void matrix_to_position_quaternion(const Eigen::Matrix4d &transform, Position &position,
                                   Rotation &quaternion) {
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

void matrix_to_position_euler(const Eigen::Matrix4d &transform, Position &position, double &roll,
                              double &pitch, double &yaw) {
    // Extract position
    position.x = transform(0, 3);
    position.y = transform(1, 3);
    position.z = transform(2, 3);

    // Extract rotation matrix and convert to Euler angles (ZYX convention)
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order

    yaw = euler_angles(0);
    pitch = euler_angles(1);
    roll = euler_angles(2);
}

Eigen::Matrix4d position_quaternion_to_matrix(const Position &position,
                                              const Rotation &quaternion) {
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

Eigen::Matrix4d position_euler_to_matrix(const Position &position, double roll, double pitch,
                                         double yaw) {
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

Eigen::Matrix4d pose_to_matrix(const Pose &pose) {
    return position_quaternion_to_matrix(pose.position, pose.rotation);
}

Pose matrix_to_pose(const Eigen::Matrix4d &transform) {
    Pose pose;
    matrix_to_position_quaternion(transform, pose.position, pose.rotation);
    return pose;
}

Position vector_to_position(const Eigen::Vector3d &vector) {
    return Position{vector[0], vector[1], vector[2]};
}

void quaternion_to_euler(const Rotation &quaternion, double &roll, double &pitch, double &yaw) {
    double w = quaternion.w, x = quaternion.x, y = quaternion.y, z = quaternion.z;
    double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 0.0) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    // Roll (X)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y) -- clamp to avoid NaN at poles
    double sinp = 2.0 * (w * y - z * x);
    sinp = std::clamp(sinp, -1.0, 1.0);
    pitch = std::asin(sinp);

    // Yaw (Z)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

Rotation euler_to_quaternion(double roll, double pitch, double yaw) {
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

TransformStamped invert_transform(const TransformStamped &transform) {
    TransformStamped inverted;

    // Swap parent and child frames
    inverted.header.frame_id = transform.child_frame_id;
    inverted.child_frame_id = transform.header.frame_id;

    // Copy timestamp
    inverted.header.stamp = transform.header.stamp;

    // Invert the transformation matrix
    inverted.transform.tf = transform.transform.tf.inverse();

    return inverted;
}

std::string transform_to_string(const TransformStamped &transform) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    // Extract position and quaternion from the transform matrix
    Position position;
    Rotation quaternion;
    matrix_to_position_quaternion(transform.transform.tf, position, quaternion);

    // Convert quaternion to Euler angles for readability
    double roll, pitch, yaw;
    quaternion_to_euler(quaternion, roll, pitch, yaw);

    oss << "TransformStamped(";
    oss << "parent=" << enum_to_string_lower(transform.header.frame_id);
    oss << ", child=" << enum_to_string_lower(transform.child_frame_id);
    oss << ", time=" << transform.header.stamp;
    oss << ", pos=[" << position.x << ", " << position.y << ", " << position.z << "]";
    oss << ", quat=[" << quaternion.w << ", " << quaternion.x << ", " << quaternion.y << ", "
        << quaternion.z << "]";
    oss << ", rpy=[" << roll << ", " << pitch << ", " << yaw << "]";
    oss << ")";

    return oss.str();
}

Pose2D pose_to_pose2d(const Pose &pose) {
    double roll, pitch, yaw;
    quaternion_to_euler(pose.rotation, roll, pitch, yaw);
    return Pose2D{pose.position.x, pose.position.y, yaw};
}

Pose pose2d_to_pose(const Pose2D &pose2d) {
    Pose pose;
    pose.position = Position{pose2d.x, pose2d.y, 0.0};
    pose.rotation = euler_to_quaternion(0.0, 0.0, pose2d.yaw);
    return pose;
}

Position pose2d_to_position(const Pose2D &pose2d) { return Position{pose2d.x, pose2d.y, 0.0}; }

bool pixel_to_camera_ray(const CameraInfo &camera_info, double pixel_x, double pixel_y,
                         Eigen::Vector3d &out_ray) {
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) return false;
    const double fx = camera_info.intrinsics.at<double>(0, 0);
    const double fy = camera_info.intrinsics.at<double>(1, 1);
    const double cx = camera_info.intrinsics.at<double>(0, 2);
    const double cy = camera_info.intrinsics.at<double>(1, 2);
    if (std::abs(fx) < 1e-6 || std::abs(fy) < 1e-6) return false;

    out_ray = Eigen::Vector3d((pixel_x - cx) / fx, (pixel_y - cy) / fy, 1.0);
    return std::isfinite(out_ray.x()) && std::isfinite(out_ray.y()) && std::isfinite(out_ray.z());
}

bool intersect_camera_ray_with_plane(const Eigen::Vector3d &ray,
                                     const Eigen::Vector3d &plane_center,
                                     const Eigen::Vector3d &plane_normal,
                                     Eigen::Vector3d &out_point) {
    constexpr double EPSILON = 1e-6;
    const double dot_with_normal = ray.dot(plane_normal);
    if (std::abs(dot_with_normal) < EPSILON) return false;

    const double fac = -plane_center.dot(-plane_normal) / dot_with_normal;
    if (!std::isfinite(fac) || fac <= 0.0) return false;
    out_point = ray * fac;
    return std::isfinite(out_point.x()) && std::isfinite(out_point.y()) &&
           std::isfinite(out_point.z());
}

bool transform_to_plane_center_normal(const Transform &transform, Eigen::Vector3d &out_center,
                                      Eigen::Vector3d &out_normal) {
    if (transform.tf.rows() < 4 || transform.tf.cols() < 4) return false;
    const Eigen::Matrix4d tf = transform.tf.block<4, 4>(0, 0);

    const Eigen::Vector4d origin_field(0.0, 0.0, 0.0, 1.0);
    const Eigen::Vector4d x_axis_field(1.0, 0.0, 0.0, 1.0);
    const Eigen::Vector4d y_axis_field(0.0, 1.0, 0.0, 1.0);

    out_center = (tf * origin_field).head<3>();
    const Eigen::Vector3d px = (tf * x_axis_field).head<3>();
    const Eigen::Vector3d py = (tf * y_axis_field).head<3>();

    out_normal = (px - out_center).cross(py - out_center);
    const double mag = out_normal.norm();
    if (mag < 1e-6) return false;
    out_normal /= mag;
    return std::isfinite(out_center.x()) && std::isfinite(out_center.y()) &&
           std::isfinite(out_center.z()) && std::isfinite(out_normal.x()) &&
           std::isfinite(out_normal.y()) && std::isfinite(out_normal.z());
}

}  // namespace auto_battlebot
