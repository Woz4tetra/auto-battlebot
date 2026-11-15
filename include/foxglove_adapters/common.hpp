#pragma once

#include <foxglove/schemas.hpp>
#include <string>

#include "data_structures/header.hpp"
#include "data_structures/pose.hpp"
#include "enum_to_string_lower.hpp"
#include "label_utils.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::Timestamp to_timestamp(double stamp_seconds);
foxglove::schemas::Duration to_duration(double seconds);

inline std::string frame_id_string(FrameId frame_id) { return enum_to_string_lower(frame_id); }

foxglove::schemas::Color to_color(float r, float g, float b, float a);
foxglove::schemas::Color to_color(const ColorRGBf &color, float alpha);

inline foxglove::schemas::Vector3 to_vector3(double x, double y, double z) {
    foxglove::schemas::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}
inline foxglove::schemas::Point3 to_point3(double x, double y, double z) {
    foxglove::schemas::Point3 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}
inline foxglove::schemas::Quaternion to_quaternion(double w, double x, double y, double z) {
    foxglove::schemas::Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}
foxglove::schemas::Pose to_pose(const Pose &pose);
foxglove::schemas::Pose identity_pose();
foxglove::schemas::Pose pose_at(double x, double y, double z);

std::string json_escape(const std::string &value);

/** Shortest round-trip decimal for a double, or `null` for NaN/inf, which JSON cannot carry. */
std::string json_number(double value);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
