#include "ros/ros_message_adapters/ros_detections.hpp"

#include <cstdio>
#include <string>

#include "enum_to_string_lower.hpp"

namespace auto_battlebot {
namespace ros_adapters {

std_msgs::String to_ros_detections(const DetectionsStamped &detections) {
    std::string json;
    json.reserve(64 + detections.detections.size() * 128);

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "{\"stamp\":%.9f,\"w\":%d,\"h\":%d,\"dets\":[",
                  detections.header.stamp, detections.image_width, detections.image_height);
    json += buffer;

    bool first = true;
    for (const auto &det : detections.detections) {
        if (!first) json += ',';
        first = false;
        std::snprintf(buffer, sizeof(buffer),
                      "{\"x1\":%.1f,\"y1\":%.1f,\"x2\":%.1f,\"y2\":%.1f,\"conf\":%.4f,"
                      "\"class_id\":%d,\"label\":\"",
                      det.x1, det.y1, det.x2, det.y2, det.confidence, det.class_id);
        json += buffer;
        json += enum_to_string_lower(det.label);
        json += '"';
        if (!det.keypoints.empty()) {
            json += ",\"kps\":[";
            bool first_kp = true;
            for (const auto &kp : det.keypoints) {
                if (!first_kp) json += ',';
                first_kp = false;
                std::snprintf(buffer, sizeof(buffer), "[%.1f,%.1f,%.4f]", kp.x, kp.y,
                              kp.confidence);
                json += buffer;
            }
            json += ']';
        }
        json += '}';
    }
    json += "]}";

    std_msgs::String msg;
    msg.data = std::move(json);
    return msg;
}

}  // namespace ros_adapters
}  // namespace auto_battlebot
