#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
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

struct FrameIdentity {
    uint64_t image_stamp_ns = 0;
    int64_t video_frame_index = -1;
    uint64_t recording_generation = 0;
};

struct SimRobotMeasurement {
    bool measured = false;
    Pose2D pose;
};

struct SimMeasurementFrame {
    bool delivered = false;
    double capture_stamp = 0.0;
    std::vector<SimRobotMeasurement> robots;
};

struct SimGroundTruth {
    Pose2D our_robot;
    std::vector<Pose2D> opponents;
};

struct CameraData {
    TransformStamped tf_visodom_from_camera;
    FrameIdentity frame_identity;
    bool pose_tracking_ready = true;
    CameraInfo camera_info;
    RgbImage rgb;
    DepthImage depth;
    std::optional<SimGroundTruth> ground_truth = std::nullopt;
    SimMeasurementFrame sim_measurements;
};

}  // namespace auto_battlebot
