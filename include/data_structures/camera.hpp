#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "header.hpp"
#include "image.hpp"
#include "pose.hpp"
#include "transform.hpp"

namespace auto_battlebot {
struct CameraInfo {
    Header header;
    int width = 0;
    int height = 0;
    cv::Mat intrinsics;
    cv::Mat distortion;
};

struct CameraData {
    TransformStamped tf_visodom_from_camera;
    CameraInfo camera_info;
    RgbImage rgb;
    DepthImage depth;
    /** Ground truth poses from simulation (index 0 = our robot, rest = opponents). Empty outside sim. */
    std::vector<Pose2D> ground_truth_poses;
};

}  // namespace auto_battlebot
