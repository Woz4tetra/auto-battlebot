#include "field_filter/point_cloud_field_filter.hpp"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace auto_battlebot
{
    PointCloudFieldFilter::PointCloudFieldFilter(PointCloudFieldFilterConfiguration &config) : distance_threshold_(config.distance_threshold)
    {
        // Constructor implementation
    }

    void PointCloudFieldFilter::reset([[maybe_unused]] TransformStamped tf_visodom_from_camera)
    {
        // Reset implementation
    }

    FieldDescription PointCloudFieldFilter::compute_field(const CameraData &camera_data, const FieldMaskStamped &field_mask)
    {
        cv::Mat masked_depth_image = mask_depth_image(camera_data.depth.image, field_mask.mask.mask);
        pcl::PointCloud<pcl::PointXYZ>::Ptr field_cloud = create_point_cloud_from_depth(masked_depth_image, camera_data.camera_info.intrinsics);
        std::vector<int> inlier_indices;
        Eigen::Vector4f plane_coefficients = fit_plane_ransac(field_cloud, distance_threshold_, &inlier_indices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud = extract_inliers(field_cloud, &inlier_indices);
        Eigen::Vector3f plane_normal = plane_normal_from_coefficients(plane_coefficients);
        Eigen::Vector3f plane_center = plane_center_from_inliers(inlier_cloud);
        Eigen::Matrix4d plane_transform = transform_from_plane(plane_center, plane_normal);
        pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud = transform_points(inlier_cloud, plane_transform.inverse());
        std::vector<Eigen::Vector2f> flattened_cloud_2d = point_cloud_to_2d(flattened_cloud);
        std::vector<Eigen::Vector2f> rectangle_corners = find_minimum_rectangle(flattened_cloud_2d);
        Eigen::Vector2f rectangle_extents = get_rectangle_extents(rectangle_corners);
        double rectangle_angle = get_rectangle_angle(rectangle_corners);
        if (rectangle_angle > 0)
            rectangle_extents = Eigen::Vector2f(rectangle_extents[1], rectangle_extents[0]);
        Eigen::Vector2f rectangle_centroid = get_rectangle_centroid(rectangle_corners);
        Eigen::Matrix4d flat_transform = transform_from_position_and_euler(Eigen::Vector3f(rectangle_centroid[0], rectangle_centroid[1], 0.0), 0.0, 0.0, rectangle_angle);
        Eigen::Matrix4d field_centered_transform = flat_transform * plane_transform;

        FieldDescription field_description;
        Size field_size;
        field_size.x = rectangle_extents[0];
        field_size.y = rectangle_extents[1];
        field_size.z = 0.0;
        field_description.header = camera_data.depth.header;
        field_description.tf_camera_from_fieldcenter.tf = field_centered_transform;
        field_description.size.header = camera_data.depth.header;
        field_description.size.size = field_size;
        field_description.inlier_points.header = camera_data.depth.header;
        field_description.inlier_points.cloud = inlier_cloud;

        return field_description;
    }

    FieldDescription PointCloudFieldFilter::track_field([[maybe_unused]] TransformStamped tf_visodom_from_camera, [[maybe_unused]] FieldDescription initial_description)
    {
        // Track field implementation
        return FieldDescription{};
    }

    cv::Mat PointCloudFieldFilter::mask_depth_image(
        const cv::Mat &depth_image,
        const cv::Mat &mask) const
    {
        // Create a copy of the depth image
        cv::Mat masked_depth = depth_image.clone();

        // Set depth to NaN where mask is zero using OpenCV operations
        masked_depth.setTo(std::numeric_limits<float>::quiet_NaN(), mask == 0);

        return masked_depth;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFieldFilter::create_point_cloud_from_depth(
        const cv::Mat &depth_image,
        const cv::Mat &intrinsics) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Extract camera intrinsics (assuming 3x3 matrix)
        float fx = intrinsics.at<double>(0, 0);
        float fy = intrinsics.at<double>(1, 1);
        float cx = intrinsics.at<double>(0, 2);
        float cy = intrinsics.at<double>(1, 2);

        // Reserve space for point cloud
        cloud->points.reserve(depth_image.rows * depth_image.cols);

        // Convert depth image to point cloud
        for (int v = 0; v < depth_image.rows; ++v)
        {
            for (int u = 0; u < depth_image.cols; ++u)
            {
                float depth = depth_image.at<float>(v, u);

                // Skip invalid depth values
                if (std::isnan(depth) || depth <= 0.0f)
                {
                    continue;
                }

                // Back-project pixel to 3D point
                pcl::PointXYZ point;
                point.z = depth;
                point.x = (u - cx) * depth / fx;
                point.y = (v - cy) * depth / fy;

                cloud->points.push_back(point);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        return cloud;
    }

    Eigen::Vector4f PointCloudFieldFilter::fit_plane_ransac(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        double distance_threshold,
        std::vector<int> *inliers) const
    {
        // Create the plane model
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

        // Create RANSAC object
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
        ransac.setDistanceThreshold(distance_threshold);

        // Compute the model
        ransac.computeModel();

        // Get the plane coefficients [a, b, c, d] where ax + by + cz + d = 0
        Eigen::VectorXf coefficients;
        ransac.getModelCoefficients(coefficients);

        // Get inliers if requested
        if (inliers != nullptr)
        {
            ransac.getInliers(*inliers);
        }

        return coefficients;
    }

    Eigen::Vector3f PointCloudFieldFilter::plane_normal_from_coefficients(const Eigen::Vector4f &coefficients) const
    {
        // Extract normal vector from plane coefficients [a, b, c, d]
        // For plane equation ax + by + cz + d = 0, the normal is [a, b, c]
        Eigen::Vector3f normal(coefficients[0], coefficients[1], coefficients[2]);

        // Normalize the normal vector
        float magnitude = normal.norm();
        if (magnitude < 1e-6f)
        {
            // Return default normal pointing down if magnitude is too small
            return Eigen::Vector3f(0.0f, 0.0f, -1.0f);
        }

        return normal / magnitude;
    }

    Eigen::Vector3f PointCloudFieldFilter::plane_center_from_inliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud) const
    {
        Eigen::Vector3f center(0.0f, 0.0f, 0.0f);

        // Compute the centroid of inlier points
        for (const auto &point : inlier_cloud->points)
        {
            center.x() += point.x;
            center.y() += point.y;
            center.z() += point.z;
        }

        center /= static_cast<float>(inlier_cloud->size());
        return center;
    }

    Eigen::Matrix4d PointCloudFieldFilter::transform_from_plane(
        const Eigen::Vector3f &plane_center,
        const Eigen::Vector3f &plane_normal,
        const Eigen::Vector3f &up_vector) const
    {
        // Create identity transform matrix
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        // Compute rotation that aligns up_vector with plane_normal
        Eigen::Vector3d up = up_vector.cast<double>().normalized();
        Eigen::Vector3d normal = plane_normal.cast<double>().normalized();

        // Use Eigen's Quaternion to compute rotation from up to normal
        Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(up, normal);

        // Set rotation part of transform
        transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

        // Set translation part of transform
        transform.block<3, 1>(0, 3) = plane_center.cast<double>();

        return transform;
    }

    Eigen::Matrix4d PointCloudFieldFilter::transform_from_position_and_euler(
        const Eigen::Vector3f &position,
        double roll,
        double pitch,
        double yaw) const
    {
        // Create identity transform matrix
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        // Create rotation matrix from Euler angles (ZYX convention: yaw-pitch-roll)
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        // Combined rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

        // Set rotation part of transform
        transform.block<3, 3>(0, 0) = rotation;

        // Set translation part of transform
        transform.block<3, 1>(0, 3) = position.cast<double>();

        return transform;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFieldFilter::extract_inliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const std::vector<int> *inliers) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (inliers == nullptr || inliers->empty())
        {
            return inlier_cloud;
        }

        inlier_cloud->points.reserve(inliers->size());

        for (int idx : *inliers)
        {
            inlier_cloud->points.push_back(cloud->points[idx]);
        }

        inlier_cloud->width = inlier_cloud->points.size();
        inlier_cloud->height = 1;
        inlier_cloud->is_dense = true;

        return inlier_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFieldFilter::transform_points(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const Eigen::Matrix4d &transform) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        transformed_cloud->points.reserve(cloud->points.size());

        for (const auto &point : cloud->points)
        {
            // Create homogeneous coordinates [x, y, z, 1]
            Eigen::Vector4d homogeneous_point(point.x, point.y, point.z, 1.0);

            // Apply transformation
            Eigen::Vector4d transformed_homogeneous = transform * homogeneous_point;

            // Convert back to 3D point
            pcl::PointXYZ transformed_point;
            transformed_point.x = static_cast<float>(transformed_homogeneous[0]);
            transformed_point.y = static_cast<float>(transformed_homogeneous[1]);
            transformed_point.z = static_cast<float>(transformed_homogeneous[2]);

            transformed_cloud->points.push_back(transformed_point);
        }

        transformed_cloud->width = transformed_cloud->points.size();
        transformed_cloud->height = 1;
        transformed_cloud->is_dense = true;

        return transformed_cloud;
    }

    std::vector<Eigen::Vector2f> PointCloudFieldFilter::point_cloud_to_2d(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
    {
        std::vector<Eigen::Vector2f> points_2d;
        points_2d.reserve(cloud->points.size());

        for (const auto &point : cloud->points)
        {
            points_2d.emplace_back(point.x, point.y);
        }

        return points_2d;
    }

    std::vector<Eigen::Vector2f> PointCloudFieldFilter::find_minimum_rectangle(
        const std::vector<Eigen::Vector2f> &points) const
    {
        if (points.size() < 3)
        {
            return points;
        }

        // Convert to OpenCV format for convex hull computation
        std::vector<cv::Point2f> cv_points;
        cv_points.reserve(points.size());
        for (const auto &p : points)
        {
            cv_points.emplace_back(p.x(), p.y());
        }

        // Compute convex hull
        std::vector<cv::Point2f> hull;
        cv::convexHull(cv_points, hull);

        if (hull.size() < 3)
        {
            std::vector<Eigen::Vector2f> result;
            for (const auto &p : hull)
            {
                result.emplace_back(p.x, p.y);
            }
            return result;
        }

        // Calculate edge angles
        std::vector<float> angles;
        const float pi2 = M_PI / 2.0f;

        for (size_t i = 0; i < hull.size(); ++i)
        {
            size_t next = (i + 1) % hull.size();
            float dx = hull[next].x - hull[i].x;
            float dy = hull[next].y - hull[i].y;
            float angle = std::atan2(dy, dx);
            angle = std::fmod(std::abs(angle), pi2);
            angles.push_back(angle);
        }

        // Get unique angles
        std::sort(angles.begin(), angles.end());
        angles.erase(std::unique(angles.begin(), angles.end(),
                                 [](float a, float b)
                                 { return std::abs(a - b) < 1e-6f; }),
                     angles.end());

        // Find the rotation with minimum area
        float min_area = std::numeric_limits<float>::max();
        Eigen::Matrix2f best_rotation = Eigen::Matrix2f::Identity();
        float best_min_x = 0.0f, best_max_x = 0.0f, best_min_y = 0.0f, best_max_y = 0.0f;

        for (float angle : angles)
        {
            // Create rotation matrix
            Eigen::Matrix2f rotation;
            rotation << std::cos(angle), -std::sin(angle),
                std::sin(angle), std::cos(angle);

            // Rotate all hull points and find bounding box
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();

            for (const auto &p : hull)
            {
                Eigen::Vector2f point(p.x, p.y);
                Eigen::Vector2f rotated = rotation * point;
                min_x = std::min(min_x, rotated.x());
                max_x = std::max(max_x, rotated.x());
                min_y = std::min(min_y, rotated.y());
                max_y = std::max(max_y, rotated.y());
            }

            float area = (max_x - min_x) * (max_y - min_y);
            if (area < min_area)
            {
                min_area = area;
                best_rotation = rotation;
                best_min_x = min_x;
                best_max_x = max_x;
                best_min_y = min_y;
                best_max_y = max_y;
            }
        }

        // Create the four corners of the rectangle in rotated space
        std::vector<Eigen::Vector2f> box_corners = {
            {best_max_x, best_min_y},
            {best_min_x, best_min_y},
            {best_min_x, best_max_y},
            {best_max_x, best_max_y}};

        // Rotate back to original space
        std::vector<Eigen::Vector2f> result;
        Eigen::Matrix2f inverse_rotation = best_rotation.transpose();
        for (const auto &corner : box_corners)
        {
            result.push_back(inverse_rotation * corner);
        }

        return result;
    }

    Eigen::Vector2f PointCloudFieldFilter::get_rectangle_extents(std::vector<Eigen::Vector2f> rectangle_corners) const
    {
        Eigen::Vector2f diff_01 = rectangle_corners[0] - rectangle_corners[1];
        Eigen::Vector2f diff_12 = rectangle_corners[1] - rectangle_corners[2];
        double width = diff_01.norm();
        double height = diff_12.norm();
        return Eigen::Vector2f(width, height);
    }

    Eigen::Vector2f PointCloudFieldFilter::get_rectangle_centroid(
        const std::vector<Eigen::Vector2f> &rectangle_corners) const
    {
        if (rectangle_corners.empty())
        {
            return Eigen::Vector2f(0.0f, 0.0f);
        }

        // Compute the mean of all corner positions
        Eigen::Vector2f centroid(0.0f, 0.0f);
        for (const auto &corner : rectangle_corners)
        {
            centroid += corner;
        }
        centroid /= static_cast<float>(rectangle_corners.size());

        return centroid;
    }

    template <typename T>
    T PointCloudFieldFilter::input_modulus(T value, T min_value, T max_value) const
    {
        // Bound the value between min_value and max_value, wrapping around if it goes over
        T modulus = max_value - min_value;
        value -= min_value;
        value = std::fmod(value, modulus);
        if (value < static_cast<T>(0))
        {
            value += modulus;
        }
        value += min_value;
        return value;
    }

    double PointCloudFieldFilter::normalize_angle(double angle) const
    {
        // Normalize angle to [-pi, pi] using input_modulus
        return input_modulus(angle, -M_PI, M_PI);
    }

    double PointCloudFieldFilter::get_rectangle_angle(const std::vector<Eigen::Vector2f> &rectangle) const
    {
        if (rectangle.size() != 4)
        {
            return 0.0f;
        }

        std::vector<double> angles;
        std::vector<Eigen::Vector2f> midpoints;
        angles.reserve(4);
        midpoints.reserve(4);

        for (size_t i = 0; i < rectangle.size(); ++i)
        {
            const Eigen::Vector2f &p0 = rectangle[i];
            const Eigen::Vector2f &p1 = rectangle[(i + 1) % rectangle.size()];
            Eigen::Vector2f delta = p1 - p0;
            double angle = std::atan2(delta.y(), delta.x());
            angles.push_back(angle);
            midpoints.push_back((p0 + p1) / 2.0f);
        }

        // Find the edge with the minimum y midpoint
        size_t min_y_index = 0;
        float min_y = midpoints[0].y();
        for (size_t i = 1; i < midpoints.size(); ++i)
        {
            if (midpoints[i].y() < min_y)
            {
                min_y = midpoints[i].y();
                min_y_index = i;
            }
        }

        double angle = angles[min_y_index];
        double complementary_angle = normalize_angle(angle + M_PI);

        if (std::abs(complementary_angle) < std::abs(angle))
        {
            angle = complementary_angle;
        }

        return angle;
    }

} // namespace auto_battlebot
