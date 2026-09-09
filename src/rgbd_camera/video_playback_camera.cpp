#include "rgbd_camera/video_playback_camera.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mcap/reader.hpp>
#include <thread>

#include "config/config_parser.hpp"
#include "directories.hpp"

namespace auto_battlebot {
struct VideoPlaybackCamera::ReaderState {
    mcap::McapReader reader;
    std::unique_ptr<mcap::LinearMessageView> messages;
    std::unique_ptr<mcap::LinearMessageView::Iterator> cursor;
};

namespace {
constexpr const char *kVideoTopic = "/camera/video";
// How far behind real time playback will try to make up. Beyond this the deadline is re-anchored,
// so a process that was suspended or stopped at a breakpoint resumes instead of fast-forwarding
// through everything it owes.
constexpr auto kMaxCatchUp = std::chrono::milliseconds(500);
constexpr const char *kFrameMetaTopic = "/camera/frame_meta";
constexpr const char *kImageTopic = "/camera/image";

/** The `data` field of a foxglove.CompressedVideo, without linking protobuf.
 *
 * The message is four fields: timestamp(1), frame_id(2), data(3), format(4). Pulling one
 * length-delimited field out of a wire-format message is a dozen lines; a protobuf dependency for
 * it would be the larger thing to maintain. */
bool compressed_video_payload(const std::byte *message, size_t len, const std::byte *&out,
                              size_t &out_len) {
    size_t offset = 0;
    while (offset < len) {
        uint64_t key = 0;
        int shift = 0;
        while (offset < len) {
            const auto byte = static_cast<uint8_t>(message[offset++]);
            key |= static_cast<uint64_t>(byte & 0x7F) << shift;
            if ((byte & 0x80) == 0) {
                break;
            }
            shift += 7;
        }
        const uint32_t field = static_cast<uint32_t>(key >> 3);
        const uint32_t wire_type = static_cast<uint32_t>(key & 0x07);
        if (wire_type == 2) {
            uint64_t length = 0;
            shift = 0;
            while (offset < len) {
                const auto byte = static_cast<uint8_t>(message[offset++]);
                length |= static_cast<uint64_t>(byte & 0x7F) << shift;
                if ((byte & 0x80) == 0) {
                    break;
                }
                shift += 7;
            }
            if (offset + length > len) {
                return false;
            }
            if (field == 3) {
                out = message + offset;
                out_len = static_cast<size_t>(length);
                return true;
            }
            offset += static_cast<size_t>(length);
        } else if (wire_type == 0) {
            while (offset < len && (static_cast<uint8_t>(message[offset++]) & 0x80) != 0) {
            }
        } else if (wire_type == 5) {
            offset += 4;
        } else if (wire_type == 1) {
            offset += 8;
        } else {
            return false;
        }
    }
    return false;
}

/** One number out of the FrameMeta JSON we write ourselves. Fixed shape, so a parser is more
 *  machinery than the job needs. */
bool frame_meta_number(std::string_view json, std::string_view key, int64_t &out) {
    const size_t start = json.find(key);
    if (start == std::string_view::npos) {
        return false;
    }
    size_t i = start + key.size();
    if (i < json.size() && json[i] == '"') {
        ++i;
    }
    bool negative = false;
    if (i < json.size() && json[i] == '-') {
        negative = true;
        ++i;
    }
    int64_t value = 0;
    size_t digits = 0;
    for (; i < json.size() && json[i] >= '0' && json[i] <= '9'; ++i, ++digits) {
        value = value * 10 + (json[i] - '0');
    }
    if (digits == 0) {
        return false;
    }
    out = negative ? -value : value;
    return true;
}

std::filesystem::path resolve(const std::string &value) {
    std::filesystem::path path(value);
    if (path.is_absolute()) {
        return path;
    }
    return get_project_root() / path;
}
}  // namespace

VideoPlaybackCamera::VideoPlaybackCamera(const VideoPlaybackCameraConfiguration &config)
    : config_(config), diagnostics_logger_(DiagnosticsLogger::get_logger("video_playback_camera")) {
    video_file_path_ = resolve(config.video_file_path).string();
}

VideoPlaybackCamera::~VideoPlaybackCamera() {
    if (state_) {
        state_->cursor.reset();
        state_->messages.reset();
        state_->reader.close();
    }
}

bool VideoPlaybackCamera::scan_file() {
    // Metadata first: the recording says which calibration rectified it, so a revised calibration
    // can be applied to footage already shot.
    {
        std::ifstream stream(video_file_path_, std::ios::binary);
        mcap::FileStreamReader data_source(stream);
        mcap::TypedRecordReader record_reader(data_source, 8);
        record_reader.onMetadata = [this](const mcap::Metadata &metadata, mcap::ByteOffset) {
            const auto it = metadata.metadata.find("calibration_id");
            if (it != metadata.metadata.end()) {
                recorded_calibration_id_ = it->second;
            }
        };
        while (record_reader.next()) {
            if (!record_reader.status().ok()) {
                break;
            }
        }
    }

    const mcap::Status status = state_->reader.open(video_file_path_);
    if (!status.ok()) {
        spdlog::error("[VideoPlaybackCamera] Cannot open {}: {}", video_file_path_, status.message);
        return false;
    }
    const auto summary = state_->reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!summary.ok()) {
        spdlog::error("[VideoPlaybackCamera] Cannot read the summary of {}: {}", video_file_path_,
                      summary.message);
        return false;
    }

    bool has_image = false;
    bool has_video = false;
    for (const auto &[id, channel] : state_->reader.channels()) {
        (void)id;
        if (channel->topic == kVideoTopic) {
            has_video = true;
        } else if (channel->topic == kImageTopic) {
            has_image = true;
        }
    }
    if (!has_video) {
        // Name the missing channel. A recording that carries only /camera/image starts fine and
        // then produces nothing, which reads as a broken camera rather than the wrong recording.
        spdlog::error(
            "[VideoPlaybackCamera] {} has no {} channel{}. Playback needs the recorded video "
            "stream, not the JPEG debug topic.",
            video_file_path_, kVideoTopic,
            has_image ? std::string(" (it carries ") + kImageTopic + " instead)" : "");
        return false;
    }

    // Index the video channel once so a start_frame can seek to the last IDR at or before it.
    // Seeking within a GOP is what leaves sl::Camera::setSVOPosition returning smeared RGB, and
    // it applies to any inter-coded stream. The frame stamps are indexed in the same pass, keyed
    // on the ordinal both channels agree about.
    mcap::ReadMessageOptions index_options;
    index_options.topicFilter = [](std::string_view topic) {
        return topic == kVideoTopic || topic == kFrameMetaTopic;
    };
    for (const auto &message : state_->reader.readMessages(nullptr, index_options)) {
        if (message.channel->topic == kFrameMetaTopic) {
            const std::string_view json(reinterpret_cast<const char *>(message.message.data),
                                        message.message.dataSize);
            int64_t ordinal = -1;
            int64_t stamp = 0;
            if (frame_meta_number(json, "\"video_frame_index\":", ordinal) && ordinal >= 0 &&
                frame_meta_number(json, "\"image_stamp_ns\":", stamp)) {
                image_stamp_by_ordinal_[ordinal] = static_cast<uint64_t>(stamp);
            }
            continue;
        }
        const std::byte *payload = nullptr;
        size_t payload_len = 0;
        if (!compressed_video_payload(message.message.data, message.message.dataSize, payload,
                                      payload_len)) {
            continue;
        }
        frame_index_.push_back(VideoFrameIndex{message.message.logTime,
                                               is_keyframe_access_unit(payload, payload_len)});
    }
    if (frame_index_.empty()) {
        spdlog::error("[VideoPlaybackCamera] {} carries no decodable video frames",
                      video_file_path_);
        return false;
    }
    return true;
}

bool VideoPlaybackCamera::initialize() {
    if (config_.video_file_path.empty()) {
        throw ConfigValidationError("VideoPlaybackCamera needs a video_file_path");
    }
    if (!std::filesystem::exists(video_file_path_)) {
        spdlog::error("[VideoPlaybackCamera] {} does not exist", video_file_path_);
        return false;
    }

    state_ = std::make_unique<ReaderState>();
    if (!scan_file()) {
        return false;
    }

    if (config_.start_frame >= static_cast<int>(frame_index_.size())) {
        // Same failure the SVO path has: a scratch overlay that sets the file path but inherits
        // start_frame from the config it extends used to seek past the end and exit before the
        // first heartbeat, which reads as a corrupt recording rather than a config merge.
        throw ConfigValidationError(
            video_file_path_ + " has only " + std::to_string(frame_index_.size()) +
            " video frames, cannot start at frame " + std::to_string(config_.start_frame));
    }

    std::string calibration_file = config_.calibration_file;
    if (calibration_file.empty() && !recorded_calibration_id_.empty()) {
        calibration_file = "config/cameras/" + recorded_calibration_id_ + ".toml";
    }
    if (!calibration_file.empty()) {
        calibration_ = load_camera_calibration(calibration_file);
    } else {
        spdlog::warn(
            "[VideoPlaybackCamera] {} names no calibration_id and none is configured; frames are "
            "replayed unrectified",
            video_file_path_);
    }

    if (!decoder_.open()) {
        return false;
    }

    // Decode forward from the last keyframe at or before the target, then discard until it. With
    // an IDR every 30 frames the worst case is 29 discarded frames.
    int64_t seek_ordinal = 0;
    for (int64_t i = config_.start_frame; i >= 0; --i) {
        if (frame_index_[static_cast<size_t>(i)].keyframe) {
            seek_ordinal = i;
            break;
        }
    }
    // Video only from here. The stamps are already indexed, and camera_info is not read at all:
    // the recording is pre-rectification, so its camera_info describes a rectified stream that
    // playback regenerates from the calibration file rather than replays. Everything else in the
    // file is the previous run's output, which replay exists to regenerate.
    mcap::ReadMessageOptions options;
    options.topicFilter = [](std::string_view topic) { return topic == kVideoTopic; };
    options.startTime = frame_index_[static_cast<size_t>(seek_ordinal)].log_time_ns;
    state_->messages =
        std::make_unique<mcap::LinearMessageView>(state_->reader.readMessages(nullptr, options));
    state_->cursor = std::make_unique<mcap::LinearMessageView::Iterator>(state_->messages->begin());
    next_frame_ordinal_ = seek_ordinal;

    spdlog::info("[VideoPlaybackCamera] {}: {} video frames, starting at {} (decoding from IDR {})",
                 video_file_path_, frame_index_.size(), config_.start_frame, seek_ordinal);
    return true;
}

bool VideoPlaybackCamera::advance_to_next_frame(cv::Mat &bgr, uint64_t &log_time_ns,
                                                uint64_t &image_stamp_ns) {
    if (!state_ || !state_->cursor) {
        return false;
    }
    for (; *state_->cursor != state_->messages->end(); ++(*state_->cursor)) {
        const auto &message = **state_->cursor;
        const std::byte *payload = nullptr;
        size_t payload_len = 0;
        if (!compressed_video_payload(message.message.data, message.message.dataSize, payload,
                                      payload_len)) {
            continue;
        }
        const int64_t ordinal = next_frame_ordinal_++;
        cv::Mat decoded;
        const bool produced = decoder_.decode(payload, payload_len, decoded);
        if (ordinal < config_.start_frame || !produced) {
            continue;
        }
        bgr = decoded;
        log_time_ns = message.message.logTime;
        const auto stamp = image_stamp_by_ordinal_.find(ordinal);
        image_stamp_ns = stamp != image_stamp_by_ordinal_.end() ? stamp->second : log_time_ns;
        ++(*state_->cursor);
        return true;
    }
    return false;
}

void VideoPlaybackCamera::pace(double stamp_seconds) {
    if (!config_.real_time_mode) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!pace_anchored_) {
        pace_anchored_ = true;
        pace_anchor_ = now;
        pace_anchor_stamp_s_ = stamp_seconds;
        return;
    }

    // Sleep until the instant this frame is due, not for the interval since the last one. The
    // pipeline's work on a frame has to come out of that interval; sleeping a full interval on top
    // of it made the wall-clock period `interval + pipeline_work`, which on 59.94 fps cage footage
    // played back at 40 fps.
    const std::chrono::duration<double> offset(stamp_seconds - pace_anchor_stamp_s_);
    const auto target =
        pace_anchor_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(offset);
    if (now < target) {
        std::this_thread::sleep_until(target);
        return;
    }
    if (now - target > kMaxCatchUp) {
        // Too far behind to make up. Re-anchor rather than sprint through what is owed: a box that
        // cannot keep up should play as fast as it can, and a replay held at a breakpoint should
        // resume rather than fast-forward.
        pace_anchor_ = now;
        pace_anchor_stamp_s_ = stamp_seconds;
    }
}

bool VideoPlaybackCamera::get(CameraData &data) {
    cv::Mat bgr;
    uint64_t log_time_ns = 0;
    uint64_t image_stamp_ns = 0;
    if (!advance_to_next_frame(bgr, log_time_ns, image_stamp_ns)) {
        should_close_ = true;
        spdlog::info("[VideoPlaybackCamera] End of {}", video_file_path_);
        return false;
    }

    if (!rectifier_.ready() && !calibration_.calibration_id.empty()) {
        rectifier_.build(calibration_, bgr.size());
        camera_info_ = rectifier_.camera_info();
    }
    cv::Mat rectified;
    if (rectifier_.ready()) {
        rectifier_.apply(bgr, rectified);
    } else {
        rectified = bgr;
        camera_info_.width = bgr.cols;
        camera_info_.height = bgr.rows;
    }

    double stamp = static_cast<double>(image_stamp_ns) * 1e-9;
    if (config_.rebase_stamps) {
        if (!stamp_offset_initialized_) {
            const double now =
                static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count()) *
                1e-9;
            stamp_offset_s_ = now - stamp;
            stamp_offset_initialized_ = true;
        }
        stamp += stamp_offset_s_;
    }
    pace(stamp);

    data = CameraData{};
    data.camera_info = camera_info_;
    data.camera_info.header.stamp = stamp;
    data.camera_info.header.frame_id = FrameId::CAMERA;
    data.rgb.image = rectified;
    data.rgb.header.stamp = stamp;
    data.rgb.header.frame_id = FrameId::CAMERA;
    // depth stays empty; the RGB camera has none and playback must not invent one.
    data.tracking_ok = true;
    data.tf_visodom_from_camera.header.stamp = stamp;
    data.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
    data.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
    data.tf_visodom_from_camera.transform.tf = Eigen::Matrix4d::Identity();
    data.frame_identity.image_stamp_ns = image_stamp_ns;
    data.frame_identity.video_frame_index = next_frame_ordinal_ - 1;
    return true;
}
}  // namespace auto_battlebot
