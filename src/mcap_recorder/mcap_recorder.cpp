#include "mcap_recorder/mcap_recorder.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <foxglove/error.hpp>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace auto_battlebot {
namespace {
constexpr uint64_t kBytesPerGb = 1024ULL * 1024ULL * 1024ULL;
constexpr uint64_t kSizeCheckIntervalWrites = 2000;

// Paths a camera is reading. With video in the main MCAP a replay reads one file and records
// another, and pointing the output at the input would corrupt the source.
std::mutex& reserved_mutex() {
    static std::mutex mutex;
    return mutex;
}
std::unordered_set<std::string>& reserved_paths() {
    static std::unordered_set<std::string> paths;
    return paths;
}
}  // namespace

void McapRecorder::reserve_input_path(const std::filesystem::path& path) {
    std::error_code ec;
    const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
    std::lock_guard<std::mutex> lock(reserved_mutex());
    reserved_paths().insert((ec ? path : canonical).string());
}

McapRecorder::McapRecorder(const std::string& active_profile) : active_profile_(active_profile) {
    metadata_.emplace_back("active_profile", active_profile_);
    if (open_writer(make_file_path(active_profile_, 0))) {
        enabled_ = true;
    }
}

bool McapRecorder::open_writer(const std::filesystem::path& path) {
    {
        std::error_code ec;
        const std::filesystem::path canonical = std::filesystem::weakly_canonical(path, ec);
        std::lock_guard<std::mutex> lock(reserved_mutex());
        if (reserved_paths().count((ec ? path : canonical).string()) != 0) {
            spdlog::error(
                "[McapRecorder] Refusing to record over {}, which a camera is replaying from",
                path.string());
            return false;
        }
    }

    file_path_ = path;
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
        return false;
    }
    writer_.emplace(std::move(writer.value()));

    // Written into every segment, so each one is independently readable and says which config and
    // which camera calibration produced it.
    auto metadata_status =
        writer_->writeMetadata("auto_battlebot", metadata_.begin(), metadata_.end());
    if (metadata_status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] Failed to write metadata: {}",
                     foxglove::strerror(metadata_status));
    }

    writer_open_ = true;
    spdlog::info("[McapRecorder] Recording to {} (profile: {})", file_path_.string(),
                 active_profile_.empty() ? "<none>" : active_profile_);
    return true;
}

void McapRecorder::write_metadata(const std::string& key, const std::string& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : metadata_) {
        if (entry.first == key) {
            entry.second = value;
            return;
        }
    }
    metadata_.emplace_back(key, value);
    if (writer_open_) {
        std::vector<std::pair<std::string, std::string>> single = {{key, value}};
        auto status = writer_->writeMetadata("auto_battlebot", single.begin(), single.end());
        if (status != foxglove::FoxgloveError::Ok) {
            spdlog::warn("[McapRecorder] Failed to write {} metadata: {}", key,
                         foxglove::strerror(status));
        }
    }
}

void McapRecorder::roll_over_locked() {
    for (auto& channel : channels_) {
        if (channel.channel) channel.channel->close();
    }
    auto status = writer_->close();
    if (status != foxglove::FoxgloveError::Ok) {
        spdlog::warn("[McapRecorder] rollover close(): {}", foxglove::strerror(status));
    }
    writer_open_ = false;
    spdlog::info("[McapRecorder] Rolled over past {} GB: {}", max_size_bytes_ / kBytesPerGb,
                 file_path_.string());

    enforce_holding_dir_size_locked();
    ++segment_;
    // Channels have to be re-created against the new writer or the next segment is a file with
    // messages and no channel records.
    std::vector<ChannelSpec> specs = channel_specs_;
    channels_.clear();
    channel_by_topic_.clear();
    channel_specs_.clear();
    if (!open_writer(make_file_path(active_profile_, segment_))) {
        return;
    }
    for (const auto& spec : specs) {
        open_channel_locked(spec.topic, spec.message_encoding, spec.schema);
    }
}

void McapRecorder::check_rollover_locked() {
    if (max_size_bytes_ == 0 || !writer_open_) return;
    if (++writes_since_size_check_ < kSizeCheckIntervalWrites) return;
    writes_since_size_check_ = 0;
    std::error_code ec;
    const auto size = std::filesystem::file_size(file_path_, ec);
    if (!ec && size >= max_size_bytes_) {
        roll_over_locked();
    }
}

void McapRecorder::enforce_holding_dir_size_locked() {
    if (holding_dir_max_size_bytes_ == 0) return;
    const std::filesystem::path directory = file_path_.parent_path();
    std::error_code ec;
    if (!std::filesystem::exists(directory, ec)) return;

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) {
        return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
    });

    uintmax_t total = 0;
    for (const auto& file : files) total += std::filesystem::file_size(file, ec);
    for (const auto& file : files) {
        if (total <= holding_dir_max_size_bytes_) break;
        if (file == file_path_) continue;
        const uintmax_t size = std::filesystem::file_size(file, ec);
        if (std::filesystem::remove(file, ec)) {
            spdlog::info("[McapRecorder] Deleted the oldest recording to free space: {}",
                         file.string());
            total -= size;
        }
    }
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
    return open_channel_locked(topic, message_encoding, schema);
}

McapRecorder::ChannelId McapRecorder::open_channel_locked(const std::string& topic,
                                                          const std::string& message_encoding,
                                                          const VizSchema& schema) {
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
    channel_specs_.push_back(ChannelSpec{topic, message_encoding, schema});
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
    check_rollover_locked();
}

std::shared_ptr<McapRecorder> make_mcap_recorder(const McapRecorderConfig& config,
                                                 const std::string& active_profile) {
    if (!config.enable) return nullptr;
    auto recorder = std::make_shared<McapRecorder>(active_profile);
    recorder->set_ignored_topics(config.ignored_topics);
    recorder->set_size_limits(config.max_size_gb, config.holding_dir_max_size_gb);
    return recorder;
}

std::filesystem::path McapRecorder::make_file_path(const std::string& active_profile, int segment) {
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
    // The stamp is one-second granular, so a rollover inside the same second would land on the
    // file it just closed.
    if (segment > 0) {
        oss << "_part" << std::setfill('0') << std::setw(2) << segment;
    }
    oss << ".mcap";

    return std::filesystem::current_path() / "data" / "recordings" / oss.str();
}

}  // namespace auto_battlebot
