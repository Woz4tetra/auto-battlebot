#define MCAP_IMPLEMENTATION
#include "mcap_recorder/mcap_recorder.hpp"

#include <spdlog/spdlog.h>

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace auto_battlebot {

McapRecorder::McapRecorder(const std::string& config_name) {
    file_path_ = make_file_path(config_name);

    std::filesystem::create_directories(file_path_.parent_path());

    mcap::McapWriterOptions opts("ros1");
    opts.compression = mcap::Compression::None;
    auto status = writer_.open(file_path_.string(), opts);
    if (!status.ok()) {
        spdlog::error("[McapRecorder] Failed to open {}: {}", file_path_.string(), status.message);
        return;
    }

    writer_open_ = true;
    enabled_ = true;
    spdlog::info("[McapRecorder] Recording to {}", file_path_.string());
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
                                                 const std::filesystem::path& config_path) {
    if (!config.enable) return nullptr;
    std::string config_name = std::filesystem::canonical(config_path).stem().string();
    auto recorder = std::make_shared<McapRecorder>(config_name);
    recorder->set_ignored_topics(config.ignored_topics);
    return recorder;
}

std::filesystem::path McapRecorder::make_file_path(const std::string& config_name) {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);

    std::ostringstream oss;
    oss << "auto_battlebot_" << config_name << "_";
    oss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
    oss << ".mcap";

    return std::filesystem::current_path() / "data" / "recordings" / oss.str();
}

}  // namespace auto_battlebot
