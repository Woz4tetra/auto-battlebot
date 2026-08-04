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
 * @brief Which camera frame a pipeline output came from.
 *
 * Header::stamp is a double holding sl::TIME_REFERENCE::IMAGE, which sits roughly half a frame
 * off the timestamp the SVO recorder writes for the same grab. That makes it impossible to join
 * recorded pipeline output back to SVO frames after the fact by timestamp alone. Recording the
 * SVO file and the frame's index within it gives an exact key instead. `image_stamp_ns` keeps
 * the raw integer timestamp so the join can be validated without the double's rounding.
 */
struct FrameIdentity {
    /// Raw sl::TIME_REFERENCE::IMAGE nanoseconds, before any conversion to double seconds.
    uint64_t image_stamp_ns = 0;
    /// Index of this frame within svo_path, or -1 when SVO recording is off.
    int64_t svo_frame_index = -1;
    /// Active SVO file, or empty when SVO recording is off. Resets index on rollover.
    std::string svo_path;
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
