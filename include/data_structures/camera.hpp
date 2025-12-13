#pragma once

#include <opencv2/opencv.hpp>
#include "transform.hpp"
#include "image.hpp"

namespace auto_battlebot {

struct CameraInfo {
    int width;
    int height;
    cv::Mat intrinsics;
    cv::Mat distortion;
};

struct CameraData {
    TransformStamped tf_visodom_from_camera;
    CameraInfo camera_info;
    RgbImage rgb;
    DepthImage depth;
};

}  // namespace auto_battlebot
