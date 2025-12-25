#include "robot_filter/robot_front_back_kalman_filter.hpp"

namespace auto_battlebot
{
    RobotFrontBackKalmanFilter::RobotFrontBackKalmanFilter(RobotFrontBackKalmanFilterConfiguration &config) : config_(config)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("robot_front_back_kalman_filter");
    }

    bool RobotFrontBackKalmanFilter::initialize(const std::vector<RobotConfig> &robots)
    {
        robot_configs_.clear();
        for (const auto &robot : robots)
        {
            robot_configs_[robot.label] = robot;
        }
        return true;
    }

    RobotDescriptionsStamped RobotFrontBackKalmanFilter::update(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info)
    {
        RobotDescriptionsStamped result;
        result.header.frame_id = FrameId::FIELD;
        result.header.stamp = keypoints.header.stamp;

        Eigen::Matrix4d tf_fieldcenter_from_camera = field.tf_camera_from_fieldcenter.tf.inverse();

        std::map<Label, FrontBackAssignment> front_back_mapping = compute_front_back_mapping(keypoints, field, camera_info);
        for (const auto &[label, assignment] : front_back_mapping)
        {
            FrameId robot_frame_id = config_.label_to_frame_ids[label][0];

            std::vector<Position> keypoints;
            Eigen::Vector3d front_keypoint_in_field = transform_point(tf_fieldcenter_from_camera, assignment.front);
            Eigen::Vector3d back_keypoint_in_field = transform_point(tf_fieldcenter_from_camera, assignment.back);
            double length = (front_keypoint_in_field - back_keypoint_in_field).norm();

            Transform tf_field_from_robot;
            bool pose_found = get_pose_from_points(front_keypoint_in_field, back_keypoint_in_field, tf_field_from_robot);

            std::string label_str = std::string(magic_enum::enum_name(label));
            diagnostics_logger_->debug(label_str, {{"pose_found", pose_found ? "true" : "false"}});

            if (!pose_found)
                continue;

            result.descriptions.push_back(
                RobotDescription{
                    robot_frame_id,
                    label,
                    matrix_to_pose(tf_field_from_robot.tf),
                    Size{length, length, 0.1},
                    {vector_to_position(front_keypoint_in_field), vector_to_position(back_keypoint_in_field)}});
        }
        diagnostics_logger_->debug({{"num_robots", (int)result.descriptions.size()}});

        return result;
    }

    std::map<Label, FrontBackAssignment> RobotFrontBackKalmanFilter::compute_front_back_mapping(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info)
    {
        Pose pose_camera_from_fieldcenter = matrix_to_pose(field.tf_camera_from_fieldcenter.tf);
        Position field_pos = pose_camera_from_fieldcenter.position;
        Eigen::Vector3d plane_normal = transform_to_plane_normal(field.tf_camera_from_fieldcenter);
        Eigen::Vector3d plane_center = Eigen::Vector3d(field_pos.x, field_pos.y, field_pos.z);

        std::vector<KeypointLabel> front_keypoints = config_.front_keypoints;
        std::vector<KeypointLabel> back_keypoints = config_.back_keypoints;

        std::map<Label, FrontBackAssignment> front_back_mapping;
        for (Keypoint keypoint : keypoints.keypoints)
        {
            Eigen::Vector3d projected_keypoint = project_keypoint_onto_plane(keypoint, plane_center, plane_normal, camera_info);
            std::string keypoint_label_str = std::string(magic_enum::enum_name(keypoint.keypoint_label));
            diagnostics_logger_->debug(
                keypoint_label_str, {{"x", projected_keypoint[0]},
                                     {"y", projected_keypoint[1]},
                                     {"z", projected_keypoint[2]}});
            auto mapping_it = front_back_mapping.find(keypoint.label);
            if (mapping_it == front_back_mapping.end())
                front_back_mapping[keypoint.label] = FrontBackAssignment{};

            auto front_assign_it = std::find(front_keypoints.begin(), front_keypoints.end(), keypoint.keypoint_label);
            if (front_assign_it == front_keypoints.end())
            {
                front_back_mapping[keypoint.label].front = projected_keypoint;
                diagnostics_logger_->debug(keypoint_label_str, {{"type", "front"}});
                continue;
            }
            auto back_assign_it = std::find(back_keypoints.begin(), back_keypoints.end(), keypoint.keypoint_label);
            if (back_assign_it == back_keypoints.end())
            {
                front_back_mapping[keypoint.label].back = projected_keypoint;
                diagnostics_logger_->debug(keypoint_label_str, {{"type", "back"}});
                continue;
            }
            std::cerr << "Failed to assign keypoint with label: " << std::string(magic_enum::enum_name(keypoint.keypoint_label)) << std::endl;
        }
        return front_back_mapping;
    }

    Eigen::Vector3d RobotFrontBackKalmanFilter::project_keypoint_onto_plane(Keypoint keypoint, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal, CameraInfo camera_info)
    {
        // Extract camera intrinsics (assuming 3x3 matrix)
        double fx = camera_info.intrinsics.at<double>(0, 0);
        double fy = camera_info.intrinsics.at<double>(1, 1);
        double cx = camera_info.intrinsics.at<double>(0, 2);
        double cy = camera_info.intrinsics.at<double>(1, 2);

        double ray_x = (keypoint.x - cx) / fx;
        double ray_y = (keypoint.y - cy) / fy;
        Eigen::Vector3d ray(ray_x, ray_y, 1.0);
        Eigen::Vector3d point_on_plane = find_ray_plane_intersection(ray, plane_center, plane_normal);

        return point_on_plane;
    }

    Eigen::Vector3d RobotFrontBackKalmanFilter::plane_from_3_points(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3)
    {
        // Calculate the normal vector of the plane defined by 3 points
        Eigen::Vector3d v1 = point2 - point1;
        Eigen::Vector3d v2 = point3 - point1;
        Eigen::Vector3d normal = v1.cross(v2);
        double magnitude = normal.norm();
        if (magnitude < 1e-6)
        {
            // Degenerate case: return default normal
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        return normal / magnitude;
    }

    Eigen::Vector3d RobotFrontBackKalmanFilter::transform_to_plane_normal(Transform transform)
    {
        // Define three points in homogeneous coordinates
        Eigen::Vector3d point1(0, 0, 0);
        Eigen::Vector3d point2(1, 0, 0);
        Eigen::Vector3d point3(0, 1, 0);

        // Compute the plane normal
        Eigen::Vector3d plane_normal = plane_from_3_points(
            transform_point(transform.tf, point1),
            transform_point(transform.tf, point2),
            transform_point(transform.tf, point3));

        return plane_normal;
    }
    Eigen::Vector3d RobotFrontBackKalmanFilter::transform_point(Eigen::Matrix4d tf, Eigen::Vector3d point)
    {
        Eigen::Vector4d point_homogenous(point[0], point[1], point[2], 1);
        Eigen::Vector3d transformed_point = (tf * point_homogenous).head<3>();
        return transformed_point;
    }

    Eigen::Vector3d RobotFrontBackKalmanFilter::find_ray_plane_intersection(Eigen::Vector3d ray, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal)
    {
        // Find intersection of ray (from origin, direction 'ray') with plane
        constexpr double EPSILON = 1e-6;
        double dot_with_normal = ray.dot(plane_normal);
        if (std::abs(dot_with_normal) < EPSILON)
        {
            // Ray is parallel to the plane
            return Eigen::Vector3d::Zero();
        }
        // fac = -dot(plane_point, -plane_normal) / dot(ray, plane_normal)
        double fac = -plane_center.dot(-plane_normal) / dot_with_normal;
        return ray * fac;
    }

    bool RobotFrontBackKalmanFilter::get_pose_from_points(const Eigen::Vector3d &front_point, const Eigen::Vector3d &back_point, Transform &out_transform)
    {
        constexpr double EPSILON = 1e-6;
        Eigen::Vector3d origin_vec(1.0, 0.0, 0.0);
        // Calculate the direction vector from back_point to front_point
        Eigen::Vector3d offset_vector = back_point - front_point;
        double magnitude = offset_vector.norm();
        if (magnitude <= EPSILON)
        {
            return false;
        }
        Eigen::Vector3d direction = offset_vector / magnitude;
        // Center point between front and back
        Eigen::Vector3d center = 0.5 * (front_point + back_point);
        // Compute rotation matrix that aligns origin_vec to direction
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        // Use Eigen's Quaternion for rotation between two vectors
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(origin_vec, direction);
#pragma GCC diagnostic pop
        Eigen::Matrix3d rot = q.toRotationMatrix();
        // Build 4x4 transform matrix
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf.block<3, 3>(0, 0) = rot;
        tf.block<3, 1>(0, 3) = center;
        out_transform.tf = tf;
        return true;
    }
} // namespace auto_battlebot
