#pragma once

#include "field_filter/field_filter_interface.hpp"
#include "field_filter/config.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace auto_battlebot
{
    class PointCloudFieldFilter : public FieldFilterInterface
    {
    public:
        PointCloudFieldFilter(PointCloudFieldFilterConfiguration &config);
        ~PointCloudFieldFilter() override = default;

        void reset(TransformStamped tf_visodom_from_camera) override;
        FieldDescription compute_field(const CameraData &camera_data, const FieldMaskStamped &field_mask) override;
        FieldDescription track_field(TransformStamped tf_visodom_from_camera, FieldDescription initial_description) override;

    private:
        double distance_threshold_;

        // Helper function to find largest contour and return mask with only that contour
        cv::Mat find_largest_contour_mask(const cv::Mat &mask) const;

        // Helper function to mask depth image with NaNs where mask is zero
        cv::Mat mask_depth_image(
            const cv::Mat &depth_image,
            const cv::Mat &mask) const;

        // Helper function to create point cloud from depth image
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud_from_depth(
            const cv::Mat &depth_image,
            const cv::Mat &intrinsics) const;

        // Helper function to fit a plane to a point cloud using RANSAC
        Eigen::Vector4f fit_plane_ransac(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            double distance_threshold = 0.01,
            std::vector<int> *inliers = nullptr) const;

        // Helper function to extract normalized plane normal from plane coefficients
        // Plane coefficients [a, b, c, d] represent ax + by + cz + d = 0
        Eigen::Vector3f plane_normal_from_coefficients(const Eigen::Vector4f &coefficients) const;

        // Helper function to compute plane center (centroid) from inlier points
        Eigen::Vector3f plane_center_from_inliers(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud) const;

        // Helper function to create a 3D transform from plane normal and center
        // Aligns the up vector (default [0, 0, 1]) with the plane normal
        Eigen::Matrix4d transform_from_plane(
            const Eigen::Vector3f &plane_center,
            const Eigen::Vector3f &plane_normal,
            const Eigen::Vector3f &up_vector = Eigen::Vector3f(0.0f, 0.0f, 1.0f)) const;

        // Helper function to create a 4x4 transform from position and Euler angles (roll, pitch, yaw)
        Eigen::Matrix4d transform_from_position_and_euler(
            const Eigen::Vector3f &position,
            double roll,
            double pitch,
            double yaw) const;

        // Helper function to extract inlier points into a new point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr extract_inliers(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            const std::vector<int> *inliers) const;

        // Helper function to transform points by a 4x4 transformation matrix
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform_points(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            const Eigen::Matrix4d &transform) const;

        // Helper function to convert point cloud to 2D by removing Z component
        std::vector<Eigen::Vector2f> point_cloud_to_2d(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

        // Helper function to find the minimum bounding rectangle for 2D points
        // Returns 4 corners of the minimum area rectangle
        std::vector<Eigen::Vector2f> find_minimum_rectangle(
            const std::vector<Eigen::Vector2f> &points) const;

        // Helper function to find the rectangle width and height when unrotated.
        Eigen::Vector2f get_rectangle_extents(
            std::vector<Eigen::Vector2f> rectangle_corners) const;

        // Helper function to compute the centroid (mean position) of rectangle corners
        Eigen::Vector2f get_rectangle_centroid(
            const std::vector<Eigen::Vector2f> &rectangle_corners) const;

        // Helper function to get the angle of a rectangle with respect to the x-axis
        // Rectangle should be a vector of 4 points
        double get_rectangle_angle(const std::vector<Eigen::Vector2f> &rectangle) const;

        // Helper function to bound a value between min and max with wrapping
        template <typename T>
        T input_modulus(T value, T min_value, T max_value) const;

        // Helper function to normalize angle to [-pi, pi]
        double normalize_angle(double angle) const;
    };

} // namespace auto_battlebot
