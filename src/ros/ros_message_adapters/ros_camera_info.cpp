#include "ros/ros_message_adapters/ros_camera_info.hpp"

#include <cstdio>
#include <string>

namespace auto_battlebot {
namespace ros_adapters {
namespace {
/// Escape the characters a filesystem path can legally contain but JSON cannot carry raw.
std::string escape_json(const std::string &value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char character : value) {
        if (character == '"' || character == '\\') escaped += '\\';
        escaped += character;
    }
    return escaped;
}
}  // namespace

sensor_msgs::CameraInfo to_ros_camera_info(const CameraInfo &camera_info) {
    sensor_msgs::CameraInfo ros_camera_info;
    ros_camera_info.header = to_ros_header(camera_info.header);
    ros_camera_info.height = camera_info.height;
    ros_camera_info.width = camera_info.width;

    // Convert intrinsics matrix (3x3) to K array
    if (camera_info.intrinsics.rows == 3 && camera_info.intrinsics.cols == 3) {
        for (int i = 0; i < 9; ++i) {
            ros_camera_info.K[i] = camera_info.intrinsics.at<double>(i / 3, i % 3);
        }
    }

    // Convert distortion coefficients
    if (!camera_info.distortion.empty()) {
        ros_camera_info.D.resize(camera_info.distortion.total());
        for (size_t i = 0; i < camera_info.distortion.total(); ++i) {
            ros_camera_info.D[i] = camera_info.distortion.at<double>(i);
        }
        ros_camera_info.distortion_model = "plumb_bob";
    }

    // Set R to identity (no rectification by default)
    for (int i = 0; i < 9; ++i) {
        ros_camera_info.R[i] = (i % 4 == 0) ? 1.0 : 0.0;  // Identity matrix
    }

    // Set P to [K | 0]
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            ros_camera_info.P[row * 4 + col] = ros_camera_info.K[row * 3 + col];
        }
        ros_camera_info.P[row * 4 + 3] = 0.0;  // Fourth column is zero
    }

    return ros_camera_info;
}

std_msgs::String to_ros_frame_identity(const FrameIdentity &identity) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%llu",
                  static_cast<unsigned long long>(identity.image_stamp_ns));

    std_msgs::String message;
    message.data = "{\"image_stamp_ns\":";
    message.data += buffer;
    std::snprintf(buffer, sizeof(buffer), "%lld", static_cast<long long>(identity.svo_frame_index));
    message.data += ",\"svo_frame_index\":";
    message.data += buffer;
    message.data += ",\"svo_path\":\"";
    message.data += escape_json(identity.svo_path);
    message.data += "\"}";
    return message;
}

}  // namespace ros_adapters
}  // namespace auto_battlebot
