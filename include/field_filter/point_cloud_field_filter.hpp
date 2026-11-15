#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Dense>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/config.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
class PointCloudFieldFilter : public FieldFilterInterface {
   public:
    explicit PointCloudFieldFilter(PointCloudFieldFilterConfiguration &config);
    ~PointCloudFieldFilter() override = default;

    void reset(TransformStamped tf_visodom_from_camera) override;
    std::shared_ptr<FieldDescriptionWithInlierPoints> compute_field(
        const CameraData &camera_data, const MaskStamped &field_mask) override;
    FieldDescription track_field(
        TransformStamped tf_visodom_from_camera,
        std::shared_ptr<FieldDescriptionWithInlierPoints> initial_description) override;

   private:
    double distance_threshold_;
    bool local_visualize_debug_;
    double depth_units_per_meter_;
    int ransac_max_iterations_;
    double ransac_probability_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    TransformStamped tf_visodom_from_cameraworld_;

    cv::Mat find_largest_contour_mask(const cv::Mat &mask) const;

    cv::Mat mask_depth_image(const cv::Mat &depth_image, const cv::Mat &mask) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud_from_depth(
        const cv::Mat &depth_image, const cv::Mat &intrinsics) const;

    Eigen::Vector4f fit_plane_ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                     double distance_threshold = 0.01,
                                     std::vector<int> *inliers = nullptr) const;

    Eigen::Vector3f plane_normal_from_coefficients(const Eigen::Vector4f &coefficients) const;

    Eigen::Vector3f plane_center_from_inliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud) const;

    Eigen::Matrix4d transform_from_plane(
        const Eigen::Vector3f &plane_center, const Eigen::Vector3f &plane_normal,
        const Eigen::Vector3f &up_vector = Eigen::Vector3f(0.0f, 0.0f, -1.0f)) const;

    Eigen::Matrix4d transform_from_position_and_euler(const Eigen::Vector3f &position, double roll,
                                                      double pitch, double yaw) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr extract_inliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<int> *inliers) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_points(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Matrix4d &transform) const;

    std::vector<Eigen::Vector2f> point_cloud_to_2d(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

    std::vector<Eigen::Vector2f> find_minimum_rectangle(
        const std::vector<Eigen::Vector2f> &points) const;

    Eigen::Vector2f get_rectangle_extents(std::vector<Eigen::Vector2f> rectangle_corners) const;

    Eigen::Vector2f get_rectangle_centroid(
        const std::vector<Eigen::Vector2f> &rectangle_corners) const;

    double get_rectangle_angle(const std::vector<Eigen::Vector2f> &rectangle) const;

    template <typename T>
    T input_modulus(T value, T min_value, T max_value) const;

    double normalize_angle(double angle) const;
};

}  // namespace auto_battlebot
