#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_battlebot {

struct CallScoped {
    CallScoped() = default;
    CallScoped(const CallScoped &) = delete;
    CallScoped(CallScoped &&) = delete;
    CallScoped &operator=(const CallScoped &) = delete;
    CallScoped &operator=(CallScoped &&) = delete;
};

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
    CallScoped scoped_to_the_call_that_built_it;
};

void show_stage_mosaic_blocking(const FieldFilterDebugFrame &frame);

}  // namespace auto_battlebot
