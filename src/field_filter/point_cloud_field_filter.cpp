#include "field_filter/point_cloud_field_filter.hpp"

namespace auto_battlebot
{
    PointCloudFieldFilter::PointCloudFieldFilter(PointCloudFieldFilterConfiguration &config) : distance_threshold_(config.distance_threshold),
                                                                                               local_visualize_debug_(config.local_visualize_debug),
                                                                                               depth_units_per_meter_(config.depth_units_per_meter)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("point_cloud_field_filter");
    }

    void PointCloudFieldFilter::reset(TransformStamped tf_visodom_from_camera)
    {
        tf_visodom_from_cameraworld_ = tf_visodom_from_camera;
        std::cout << "PointCloudFieldFilter reset with transform: " << transform_to_string(tf_visodom_from_camera) << std::endl;
    }

    std::shared_ptr<FieldDescriptionWithInlierPoints> PointCloudFieldFilter::compute_field(const CameraData &camera_data, const FieldMaskStamped &field_mask)
    {
        std::cout << "Running compute_field" << std::endl;
        FunctionTimer timer(diagnostics_logger_, "compute_field");

        cv::Mat largest_contour_mask = find_largest_contour_mask(field_mask.mask.mask);
        cv::Mat masked_depth_image = mask_depth_image(camera_data.depth.image, largest_contour_mask);
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
        Eigen::Matrix4d field_centered_transform = plane_transform * flat_transform;

        FieldDescriptionWithInlierPoints field_description;
        Size field_size;
        field_size.x = rectangle_extents[0];
        field_size.y = rectangle_extents[1];
        field_size.z = 0.0;
        field_description.header.stamp = camera_data.depth.header.stamp;
        field_description.header.frame_id = FrameId::CAMERA_WORLD;
        field_description.child_frame_id = FrameId::FIELD;
        field_description.tf_camera_from_fieldcenter.tf = field_centered_transform;
        field_description.size.header = camera_data.depth.header;
        field_description.size.size = field_size;
        field_description.inlier_points.header = camera_data.depth.header;
        field_description.inlier_points.cloud = inlier_cloud;

        diagnostics_logger_->debug("find_largest_contour_mask", {{"original_mask_size", std::to_string(field_mask.mask.mask.rows) + "x" + std::to_string(field_mask.mask.mask.cols)},
                                                                 {"largest_contour_area", std::to_string(cv::countNonZero(largest_contour_mask))}});
        diagnostics_logger_->debug("create_point_cloud_from_depth", {{"num_points", std::to_string(field_cloud->points.size())}});
        diagnostics_logger_->debug("field_description", {{"field_width", std::to_string(field_size.x)},
                                                         {"field_height", std::to_string(field_size.y)},
                                                         {"num_inlier_points", std::to_string(inlier_cloud->points.size())}});
        diagnostics_logger_->debug("fit_plane_ransac", {{"num_inliers", std::to_string(inlier_indices.size())},
                                                        {"plane_a", std::to_string(plane_coefficients[0])},
                                                        {"plane_b", std::to_string(plane_coefficients[1])},
                                                        {"plane_c", std::to_string(plane_coefficients[2])},
                                                        {"plane_d", std::to_string(plane_coefficients[3])}});
        diagnostics_logger_->debug("plane_properties", {{"plane_normal_x", std::to_string(plane_normal[0])},
                                                        {"plane_normal_y", std::to_string(plane_normal[1])},
                                                        {"plane_normal_z", std::to_string(plane_normal[2])},
                                                        {"plane_center_x", std::to_string(plane_center[0])},
                                                        {"plane_center_y", std::to_string(plane_center[1])},
                                                        {"plane_center_z", std::to_string(plane_center[2])}});
        diagnostics_logger_->debug("find_minimum_rectangle", {{"num_corners", std::to_string(rectangle_corners.size())}});
        diagnostics_logger_->debug("rectangle_properties", {{"width", std::to_string(rectangle_extents[0])},
                                                            {"height", std::to_string(rectangle_extents[1])},
                                                            {"angle_rad", std::to_string(rectangle_angle)}});

        if (local_visualize_debug_)
        {
            visualize_debug_mosaic(field_mask.mask.mask, largest_contour_mask, masked_depth_image);
        }

        std::cout << "compute_field complete" << std::endl;
        return std::make_shared<FieldDescriptionWithInlierPoints>(field_description);
    }

    FieldDescription PointCloudFieldFilter::track_field(TransformStamped tf_visodom_from_camera, std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description)
    {
        if (tf_visodom_from_cameraworld_.header.frame_id == FrameId::EMPTY)
        {
            // filter isn't initialized
            return FieldDescription{};
        }
        Eigen::MatrixXd tf_cameraworld_from_camera = tf_visodom_from_cameraworld_.transform.tf.inverse() * tf_visodom_from_camera.transform.tf;
        FieldDescription next_field_description{};
        next_field_description.header.stamp = tf_visodom_from_camera.header.stamp;
        next_field_description.header.frame_id = FrameId::CAMERA;
        next_field_description.child_frame_id = initial_description->child_frame_id;
        next_field_description.size = initial_description->size;
        Transform tf_cameraworld_from_fieldcenter = initial_description->tf_camera_from_fieldcenter;
        next_field_description.tf_camera_from_fieldcenter = Transform{tf_cameraworld_from_camera.inverse() * tf_cameraworld_from_fieldcenter.tf};
        return next_field_description;
    }

    cv::Mat PointCloudFieldFilter::find_largest_contour_mask(const cv::Mat &mask) const
    {
        FunctionTimer timer(diagnostics_logger_, "find_largest_contour_mask");
        // Find all contours in the mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            return cv::Mat::zeros(mask.size(), mask.type());
        }

        // Find the contour with the largest area
        double max_area = 0.0;
        size_t max_area_idx = 0;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = cv::contourArea(contours[i]);
            if (area > max_area)
            {
                max_area = area;
                max_area_idx = i;
            }
        }

        // Create a new mask with only the largest contour
        cv::Mat result = cv::Mat::zeros(mask.size(), mask.type());
        cv::drawContours(result, contours, static_cast<int>(max_area_idx), cv::Scalar(255), cv::FILLED);

        diagnostics_logger_->debug({{"original_mask_size", std::to_string(mask.rows) + "x" + std::to_string(mask.cols)},
                                    {"original_contour_area", std::to_string(cv::countNonZero(mask))},
                                    {"largest_mask_size", std::to_string(result.rows) + "x" + std::to_string(result.cols)},
                                    {"largest_contour_area", std::to_string(cv::countNonZero(result))}});

        return result;
    }

    cv::Mat PointCloudFieldFilter::mask_depth_image(
        const cv::Mat &depth_image,
        const cv::Mat &mask) const
    {
        FunctionTimer timer(diagnostics_logger_, "mask_depth_image");
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
        FunctionTimer timer(diagnostics_logger_, "create_point_cloud_from_depth");
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
                double depth = (double)depth_image.at<float>(v, u) / depth_units_per_meter_;

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

        diagnostics_logger_->debug({{"num_points", std::to_string(cloud->points.size())}});

        return cloud;
    }

    Eigen::Vector4f PointCloudFieldFilter::fit_plane_ransac(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        double distance_threshold,
        std::vector<int> *inliers) const
    {
        FunctionTimer timer(diagnostics_logger_, "fit_plane_ransac");

        // Create the plane model
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

        // Create RANSAC object
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
        ransac.setDistanceThreshold(distance_threshold);

        // Set maximum iterations to prevent infinite loops (sklearn default ~100-1000)
        ransac.setMaxIterations(100);

        // Set probability of finding a good model (enables early termination)
        // 0.99 means 99% confidence of finding inliers-only sample
        ransac.setProbability(0.99);

        // Compute the model
        ransac.computeModel();

        // Get the plane coefficients [a, b, c, d] where ax + by + cz + d = 0
        Eigen::VectorXf coefficients;
        ransac.getModelCoefficients(coefficients);

        // Get inliers if requested
        if (inliers != nullptr)
        {
            ransac.getInliers(*inliers);
            diagnostics_logger_->debug({{"num_inliers", (int)inliers->size()}});
        }
        else
        {
            diagnostics_logger_->debug({{"num_inliers", 0}});
        }

        diagnostics_logger_->debug({{"plane_a", (double)coefficients[0]},
                                    {"plane_b", (double)coefficients[1]},
                                    {"plane_c", (double)coefficients[2]},
                                    {"plane_d", (double)coefficients[3]}});
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
            // Return default normal pointing toward camera (negative Z in camera frame)
            return Eigen::Vector3f(0.0f, 0.0f, -1.0f);
        }

        normal = normal / magnitude;

        // Ensure normal points toward the camera (negative Z direction)
        // RANSAC plane normals can point either way, so we flip if needed
        // Camera looks down the +Z axis, so field normal should point back (-Z)
        if (normal.z() < 0.0f)
        {
            normal = -normal;
        }

        return normal;
    }

    Eigen::Vector3f PointCloudFieldFilter::plane_center_from_inliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud) const
    {
        FunctionTimer timer(diagnostics_logger_, "plane_center_from_inliers");

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

        // Suppress false positive warning from GCC about uninitialized memory in Eigen's FromTwoVectors
        // The warning occurs in Eigen's internal SVD computation but the memory is actually properly initialized
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        // Use Eigen's Quaternion to compute rotation from up to normal
        Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(up, normal);
#pragma GCC diagnostic pop

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
        FunctionTimer timer(diagnostics_logger_, "extract_inliers");

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
        FunctionTimer timer(diagnostics_logger_, "transform_points");

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
        FunctionTimer timer(diagnostics_logger_, "point_cloud_to_2d");

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
        FunctionTimer timer(diagnostics_logger_, "find_minimum_rectangle");

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

        diagnostics_logger_->debug({{"num_corners", (int)result.size()}});

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
        FunctionTimer timer(diagnostics_logger_, "get_rectangle_angle");

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

    void PointCloudFieldFilter::visualize_debug_mosaic(
        const cv::Mat &original_mask,
        const cv::Mat &largest_contour_mask,
        const cv::Mat &masked_depth_image) const
    {
        // Create debug visualization mosaic
        cv::Mat original_mask_vis, largest_contour_vis, masked_depth_vis;

        // Convert masks to 3-channel for visualization
        cv::cvtColor(original_mask, original_mask_vis, cv::COLOR_GRAY2BGR);
        cv::cvtColor(largest_contour_mask, largest_contour_vis, cv::COLOR_GRAY2BGR);

        // Normalize masked depth for visualization
        cv::Mat depth_normalized;
        cv::normalize(masked_depth_image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(depth_normalized, masked_depth_vis, cv::COLORMAP_JET);

        // Replace NaN visualization with black
        for (int v = 0; v < masked_depth_image.rows; ++v)
        {
            for (int u = 0; u < masked_depth_image.cols; ++u)
            {
                if (std::isnan(masked_depth_image.at<float>(v, u)))
                {
                    masked_depth_vis.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
                }
            }
        }

        // Add labels to each image
        cv::putText(original_mask_vis, "Original Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(largest_contour_vis, "Largest Contour", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(masked_depth_vis, "Masked Depth", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Create mosaic (1x3 layout)
        cv::Mat mosaic;
        cv::hconcat(std::vector<cv::Mat>{original_mask_vis, largest_contour_vis, masked_depth_vis}, mosaic);

        cv::imshow("Field Filter Debug", mosaic);
        cv::waitKey(-1);
        cv::destroyAllWindows();
    }

} // namespace auto_battlebot
