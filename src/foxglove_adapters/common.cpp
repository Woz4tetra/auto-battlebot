#include "foxglove_adapters/common.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::Timestamp to_timestamp(double stamp_seconds) {
    foxglove::schemas::Timestamp ts;
    if (!(stamp_seconds > 0.0)) return ts;
    const double whole = std::floor(stamp_seconds);
    ts.sec = static_cast<uint32_t>(whole);
    const auto nsec = static_cast<int64_t>(std::llround((stamp_seconds - whole) * 1e9));
    ts.nsec = static_cast<uint32_t>(std::clamp<int64_t>(nsec, 0, 999999999));
    return ts;
}

foxglove::schemas::Duration to_duration(double seconds) {
    foxglove::schemas::Duration d;
    const double whole = std::floor(seconds);
    d.sec = static_cast<int32_t>(whole);
    const auto nsec = static_cast<int64_t>(std::llround((seconds - whole) * 1e9));
    d.nsec = static_cast<uint32_t>(std::clamp<int64_t>(nsec, 0, 999999999));
    return d;
}

foxglove::schemas::Color to_color(float r, float g, float b, float a) {
    foxglove::schemas::Color c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

foxglove::schemas::Color to_color(const ColorRGBf &color, float alpha) {
    return to_color(color.r, color.g, color.b, alpha);
}

foxglove::schemas::Pose to_pose(const Pose &pose) {
    foxglove::schemas::Pose out;
    out.position = to_vector3(pose.position.x, pose.position.y, pose.position.z);
    out.orientation =
        to_quaternion(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    return out;
}

foxglove::schemas::Pose identity_pose() {
    foxglove::schemas::Pose out;
    out.position = to_vector3(0.0, 0.0, 0.0);
    out.orientation = to_quaternion(1.0, 0.0, 0.0, 0.0);
    return out;
}

foxglove::schemas::Pose pose_at(double x, double y, double z) {
    foxglove::schemas::Pose out;
    out.position = to_vector3(x, y, z);
    out.orientation = to_quaternion(1.0, 0.0, 0.0, 0.0);
    return out;
}

std::string json_escape(const std::string &value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const unsigned char c : value) {
        switch (c) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    escaped += buf;
                } else {
                    escaped += static_cast<char>(c);
                }
        }
    }
    return escaped;
}

std::string json_number(double value) {
    if (!std::isfinite(value)) return "null";
    char buf[32];
    auto result = std::to_chars(buf, buf + sizeof(buf), value);
    return std::string(buf, result.ptr);
}

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
