#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

// Socket framing shared by the app-side VizSink and the viz_relay process. The full layout is
// documented in docs/foxglove_recording_format.md ("Relay socket framing"). Every frame on the
// SOCK_STREAM unix socket is [u32 len][u8 kind][body], little-endian, with len counting the kind
// byte plus the body.

namespace auto_battlebot {
namespace viz {

enum class FrameKind : uint8_t {
    ADVERTISE = 0,
    MESSAGE = 1,
    SUBSCRIBER_COUNT = 2,
};

constexpr size_t kFrameHeaderBytes = 4;
// A single /field_points message is a few MB; anything past this is a corrupt stream.
constexpr uint32_t kMaxFrameBytes = 64u * 1024u * 1024u;

struct AdvertiseFrame {
    uint32_t channel_id = 0;
    bool latch = false;
    std::string topic;
    std::string message_encoding;
    std::string schema_name;
    std::string schema_encoding;
    std::string schema_data;
};

struct MessageFrame {
    uint32_t channel_id = 0;
    uint64_t log_time_ns = 0;
    // Points into the buffer the frame was decoded from.
    const std::byte* payload = nullptr;
    size_t payload_len = 0;
};

struct SubscriberCountFrame {
    uint32_t channel_id = 0;
    uint32_t count = 0;
};

namespace detail {
inline void put_u32(std::vector<std::byte>& out, uint32_t value) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::byte>((value >> (8 * i)) & 0xff));
}
inline void put_u64(std::vector<std::byte>& out, uint64_t value) {
    for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::byte>((value >> (8 * i)) & 0xff));
}
inline void put_bytes(std::vector<std::byte>& out, const void* data, size_t len) {
    put_u32(out, static_cast<uint32_t>(len));
    const auto* bytes = static_cast<const std::byte*>(data);
    out.insert(out.end(), bytes, bytes + len);
}
inline bool get_u32(const std::byte* data, size_t len, size_t& off, uint32_t& value) {
    if (off + 4 > len) return false;
    value = 0;
    for (int i = 0; i < 4; ++i) value |= static_cast<uint32_t>(data[off + i]) << (8 * i);
    off += 4;
    return true;
}
inline bool get_u64(const std::byte* data, size_t len, size_t& off, uint64_t& value) {
    if (off + 8 > len) return false;
    value = 0;
    for (int i = 0; i < 8; ++i) value |= static_cast<uint64_t>(data[off + i]) << (8 * i);
    off += 8;
    return true;
}
inline bool get_string(const std::byte* data, size_t len, size_t& off, std::string& value) {
    uint32_t n = 0;
    if (!get_u32(data, len, off, n)) return false;
    if (off + n > len) return false;
    value.assign(reinterpret_cast<const char*>(data + off), n);
    off += n;
    return true;
}
// Writes the [u32 len] prefix once the body is complete.
inline void finish_frame(std::vector<std::byte>& out) {
    const uint32_t len = static_cast<uint32_t>(out.size() - kFrameHeaderBytes);
    for (int i = 0; i < 4; ++i) out[i] = static_cast<std::byte>((len >> (8 * i)) & 0xff);
}
inline void begin_frame(std::vector<std::byte>& out, FrameKind kind) {
    out.clear();
    out.resize(kFrameHeaderBytes);
    out.push_back(static_cast<std::byte>(kind));
}
}  // namespace detail

inline std::vector<std::byte> encode_advertise(const AdvertiseFrame& frame) {
    std::vector<std::byte> out;
    detail::begin_frame(out, FrameKind::ADVERTISE);
    detail::put_u32(out, frame.channel_id);
    out.push_back(static_cast<std::byte>(frame.latch ? 1 : 0));
    detail::put_bytes(out, frame.topic.data(), frame.topic.size());
    detail::put_bytes(out, frame.message_encoding.data(), frame.message_encoding.size());
    detail::put_bytes(out, frame.schema_name.data(), frame.schema_name.size());
    detail::put_bytes(out, frame.schema_encoding.data(), frame.schema_encoding.size());
    detail::put_bytes(out, frame.schema_data.data(), frame.schema_data.size());
    detail::finish_frame(out);
    return out;
}

inline std::vector<std::byte> encode_message(uint32_t channel_id, uint64_t log_time_ns,
                                             const std::byte* payload, size_t payload_len) {
    std::vector<std::byte> out;
    out.reserve(kFrameHeaderBytes + 1 + 4 + 8 + payload_len);
    detail::begin_frame(out, FrameKind::MESSAGE);
    detail::put_u32(out, channel_id);
    detail::put_u64(out, log_time_ns);
    out.insert(out.end(), payload, payload + payload_len);
    detail::finish_frame(out);
    return out;
}

inline std::vector<std::byte> encode_subscriber_count(uint32_t channel_id, uint32_t count) {
    std::vector<std::byte> out;
    detail::begin_frame(out, FrameKind::SUBSCRIBER_COUNT);
    detail::put_u32(out, channel_id);
    detail::put_u32(out, count);
    detail::finish_frame(out);
    return out;
}

// `body` is the frame after the [u32 len] prefix: kind byte followed by the body.
inline std::optional<FrameKind> frame_kind(const std::byte* body, size_t len) {
    if (len < 1) return std::nullopt;
    const auto kind = static_cast<uint8_t>(body[0]);
    if (kind > static_cast<uint8_t>(FrameKind::SUBSCRIBER_COUNT)) return std::nullopt;
    return static_cast<FrameKind>(kind);
}

inline bool decode_advertise(const std::byte* body, size_t len, AdvertiseFrame& out) {
    size_t off = 1;
    if (!detail::get_u32(body, len, off, out.channel_id)) return false;
    if (off + 1 > len) return false;
    out.latch = static_cast<uint8_t>(body[off++]) != 0;
    return detail::get_string(body, len, off, out.topic) &&
           detail::get_string(body, len, off, out.message_encoding) &&
           detail::get_string(body, len, off, out.schema_name) &&
           detail::get_string(body, len, off, out.schema_encoding) &&
           detail::get_string(body, len, off, out.schema_data);
}

inline bool decode_message(const std::byte* body, size_t len, MessageFrame& out) {
    size_t off = 1;
    if (!detail::get_u32(body, len, off, out.channel_id)) return false;
    if (!detail::get_u64(body, len, off, out.log_time_ns)) return false;
    out.payload = body + off;
    out.payload_len = len - off;
    return true;
}

inline bool decode_subscriber_count(const std::byte* body, size_t len, SubscriberCountFrame& out) {
    size_t off = 1;
    return detail::get_u32(body, len, off, out.channel_id) &&
           detail::get_u32(body, len, off, out.count);
}

// Default socket path. A fixed /tmp path rather than $XDG_RUNTIME_DIR so the systemd unit for
// the relay and the one for the app agree without sharing environment, and so the docker
// playback container (which runs both) needs no extra mount.
inline std::string default_socket_path() { return "/tmp/auto_battlebot_viz.sock"; }

}  // namespace viz
}  // namespace auto_battlebot
