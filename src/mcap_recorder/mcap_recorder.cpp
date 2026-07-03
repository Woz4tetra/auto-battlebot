#define MCAP_IMPLEMENTATION
#include "mcap_recorder/mcap_recorder.hpp"

#include <spdlog/spdlog.h>

#include <cctype>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace auto_battlebot {

McapRecorder::McapRecorder(const std::string& active_profile) : active_profile_(active_profile) {
    file_path_ = make_file_path(active_profile_);

    std::filesystem::create_directories(file_path_.parent_path());

    mcap::McapWriterOptions opts("ros1");
    opts.compression = mcap::Compression::None;
    auto status = writer_.open(file_path_.string(), opts);
    if (!status.ok()) {
        spdlog::error("[McapRecorder] Failed to open {}: {}", file_path_.string(), status.message);
        return;
    }

    // Record the active profile as a file-level metadata record so playback/analysis can recover
    // which config produced the recording.
    mcap::Metadata profile_metadata;
    profile_metadata.name = "auto_battlebot";
    profile_metadata.metadata["active_profile"] = active_profile_;
    auto metadata_status = writer_.write(profile_metadata);
    if (!metadata_status.ok()) {
        spdlog::warn("[McapRecorder] Failed to write active_profile metadata: {}",
                     metadata_status.message);
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
        writer_.close();
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
