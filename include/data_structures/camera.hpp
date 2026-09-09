#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
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

/**
 * @brief Which camera frame a piece of pipeline output came from.
 *
 * `Header::stamp` is a double, so it cannot carry a nanosecond stamp losslessly; `image_stamp_ns`
 * keeps the raw integer, which is what relates a video frame to the pipeline messages around it
 * without decoding anything. `video_frame_index` is the ordinal of the frame within the video
 * stream, so `start_frame` on playback has an unambiguous meaning across a rollover boundary.
 *
 * There is no path field. Video now lives on `/camera/video` inside the same MCAP as the pipeline
 * output, so there is no second file to join to.
 */
struct FrameIdentity {
    uint64_t image_stamp_ns = 0;
    int64_t video_frame_index = -1;
};

struct CameraData {
    TransformStamped tf_visodom_from_camera;
    FrameIdentity frame_identity;
    // True when camera pose tracking is ready for stable field initialization.
    bool tracking_ok = true;
    CameraInfo camera_info;
    RgbImage rgb;
    DepthImage depth;
    /** Ground truth poses from simulation (index 0 = our robot, rest = opponents). Empty outside
     * sim. */
    std::vector<Pose2D> ground_truth_poses;
};

}  // namespace auto_battlebot
