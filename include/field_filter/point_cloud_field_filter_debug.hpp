#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_battlebot {

// Every intermediate PointCloudFieldFilter::compute_field produces, in pipeline order.
// The members are references into compute_field's locals, so a frame is only valid for
// the duration of the call that built it.
struct FieldFilterDebugFrame {
    const cv::Mat &original_mask;
    const cv::Mat &largest_contour_mask;
    const cv::Mat &masked_depth_image;
    const cv::Mat &intrinsics;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &field_cloud;
    const std::vector<int> &inlier_indices;
    const std::vector<Eigen::Vector2f> &flattened_cloud_2d;
    const std::vector<Eigen::Vector2f> &rectangle_corners;
    const Eigen::Matrix4d &plane_transform;
    double rectangle_angle;
};

// Render one labeled panel per pipeline stage, tile them into a mosaic, and show it.
// Blocks until a key is pressed.
void visualize_debug_mosaic(const FieldFilterDebugFrame &frame);

}  // namespace auto_battlebot
