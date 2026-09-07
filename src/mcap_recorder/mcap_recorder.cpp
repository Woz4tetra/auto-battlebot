#include "mcap_recorder/mcap_recorder.hpp"

#include <spdlog/spdlog.h>

#include <cctype>
#include <ctime>
#include <filesystem>
#include <foxglove/error.hpp>
#include <iomanip>
#include <sstream>
#include <utility>

namespace auto_battlebot {

McapRecorder::McapRecorder(const std::string& active_profile) : active_profile_(active_profile) {
    file_path_ = make_file_path(active_profile_);

    std::filesystem::create_directories(file_path_.parent_path());

    context_ = foxglove::Context::create();
    // McapWriterOptions holds string_views, so the path string has to outlive create().
    const std::string path_string = file_path_.string();
    foxglove::McapWriterOptions opts;
    opts.context = context_;
    opts.path = path_string;
    // No compression: the Jetson writes this at 60 Hz next to the control loop.
    opts.compression = foxglove::McapCompression::None;
    auto writer = foxglove::McapWriter::create(opts);
    if (!writer.has_value()) {
        spdlog::error("[McapRecorder] Failed to open {}: {}", file_path_.string(),
                      foxglove::strerror(writer.error()));
        return;
    }
    writer_.emplace(std::move(writer.value()));

    // Record the active profile as a file-level metadata record so playback/analysis can recover
    // which config produced the recording.
    std::vector<std::pair<std::string, std::string>> profile_metadata = {
        {"active_profile", active_profile_}};
    auto metadata_status =
        writer_->writeMetadata("auto_battlebot", profile_metadata.begin(), profile_metadata.end());
    if (metadata_status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] Failed to write active_profile metadata: {}",
                     foxglove::strerror(metadata_status));
    }

    writer_open_ = true;
    enabled_ = true;
    spdlog::info("[McapRecorder] Recording to {} (profile: {})", file_path_.string(),
                 active_profile_.empty() ? "<none>" : active_profile_);
}

McapRecorder::~McapRecorder() { close(); }

void McapRecorder::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (writer_open_) {
        for (auto& channel : channels_) {
            if (channel.channel) channel.channel->close();
        }
        auto status = writer_->close();
        if (status != foxglove::FoxgloveError::Ok) {
            spdlog::warn("[McapRecorder] close(): {}", foxglove::strerror(status));
        }
        writer_open_ = false;
        enabled_ = false;
    }
}

bool McapRecorder::set_enabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled && !writer_open_) {
        enabled_ = false;
        return false;
    }
    enabled_ = enabled;
    return true;
}

bool McapRecorder::is_enabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return writer_open_ && enabled_;
}

McapRecorder::ChannelId McapRecorder::open_channel(const std::string& topic,
                                                   const std::string& message_encoding,
                                                   const VizSchema& schema) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_open_) return 0;
    if (auto it = channel_by_topic_.find(topic); it != channel_by_topic_.end()) {
        return it->second;
    }
    std::optional<foxglove::Schema> sdk_schema;
    if (!schema.name.empty()) sdk_schema = schema.to_sdk();
    auto created = foxglove::RawChannel::create(topic, message_encoding, sdk_schema, context_);
    if (!created.has_value()) {
        spdlog::error("[McapRecorder] Failed to open channel {}: {}", topic,
                      foxglove::strerror(created.error()));
        return 0;
    }
    channels_.push_back(
        Channel{topic, std::make_unique<foxglove::RawChannel>(std::move(created.value()))});
    const ChannelId id = static_cast<ChannelId>(channels_.size());
    channel_by_topic_[topic] = id;
    return id;
}

void McapRecorder::write(ChannelId channel, const std::byte* data, size_t len,
                         uint64_t log_time_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (channel == 0 || channel > channels_.size()) return;
    const Channel& entry = channels_[channel - 1];
    if (!records_topic_locked(entry.topic)) return;
    auto status = entry.channel->log(data, len, log_time_ns);
    if (status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] write {} failed: {}", entry.topic, foxglove::strerror(status));
    }
}

std::shared_ptr<McapRecorder> make_mcap_recorder(const McapRecorderConfig& config,
                                                 const std::string& active_profile) {
    if (!config.enable) return nullptr;
    auto recorder = std::make_shared<McapRecorder>(active_profile);
    recorder->set_ignored_topics(config.ignored_topics);
    return recorder;
}

std::filesystem::path McapRecorder::make_file_path(const std::string& active_profile) {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);

    // Sanitize the profile id for use in a filename (subdir profiles contain '/').
    std::string profile_slug = active_profile.empty() ? "unknown" : active_profile;
    for (char& c : profile_slug) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '.' && c != '-') c = '_';
    }

    std::ostringstream oss;
    oss << "auto_battlebot_" << profile_slug << "_";
    oss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
    oss << ".mcap";

    return std::filesystem::current_path() / "data" / "recordings" / oss.str();
}

}  // namespace auto_battlebot
