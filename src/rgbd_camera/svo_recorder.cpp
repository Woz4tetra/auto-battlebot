#include "rgbd_camera/svo_recorder.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace auto_battlebot {

namespace {
constexpr uint64_t kSizeCheckIntervalFrames = 100;
}  // namespace

std::filesystem::path unused_svo_path(const std::filesystem::path &dir, const std::string &stamp) {
    std::filesystem::path path = dir / (stamp + ".svo2");
    std::error_code ec;
    for (int part = 1; std::filesystem::exists(path, ec); ++part) {
        std::ostringstream name;
        name << stamp << "_part" << std::setfill('0') << std::setw(2) << part << ".svo2";
        path = dir / name.str();
    }
    return path;
}

bool ZedSvoRecordingTarget::is_open() const { return camera_.isOpened(); }

bool ZedSvoRecordingTarget::start_recording(const std::string &path, std::string &error) {
    sl::RecordingParameters rec_params;
    rec_params.video_filename = sl::String(path.c_str());
    rec_params.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
    const sl::ERROR_CODE err = camera_.enableRecording(rec_params);
    if (err == sl::ERROR_CODE::SUCCESS) return true;
    error = sl::toString(err).c_str();
    return false;
}

void ZedSvoRecordingTarget::stop_recording() { camera_.disableRecording(); }

SvoRecorder::SvoRecorder(std::unique_ptr<SvoRecordingTarget> target,
                         std::filesystem::path holding_dir, uint64_t max_size_bytes,
                         uint64_t holding_dir_max_size_bytes)
    : target_(std::move(target)),
      holding_dir_(std::move(holding_dir)),
      max_size_bytes_(max_size_bytes),
      holding_dir_max_size_bytes_(holding_dir_max_size_bytes) {}

SvoRecorder::SvoRecorder(sl::Camera &camera, std::filesystem::path holding_dir,
                         uint64_t max_size_bytes, uint64_t holding_dir_max_size_bytes)
    : SvoRecorder(std::make_unique<ZedSvoRecordingTarget>(camera), std::move(holding_dir),
                  max_size_bytes, holding_dir_max_size_bytes) {}

void SvoRecorder::set_desired(bool enabled) { desired_.store(enabled); }

bool SvoRecorder::desired() const { return desired_.load(); }

bool SvoRecorder::start() {
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_);
    return start_locked(lifecycle);
}

void SvoRecorder::stop() {
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_);
    stop_locked(lifecycle);
}

bool SvoRecorder::start_locked(const std::unique_lock<std::mutex> &) {
    if (!target_->is_open()) {
        spdlog::error("Cannot start SVO recording: the camera is not open");
        return false;
    }
    std::filesystem::create_directories(holding_dir_);
    enforce_holding_dir_size();

    std::string path = generate_filename();
    std::string error;
    if (!target_->start_recording(path, error)) {
        spdlog::error("Failed to start SVO recording: {}", error);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_path_ = path;
        frame_index_ = -1;
    }
    desired_.store(true);
    spdlog::info("SVO recording started: {}", path);
    return true;
}

void SvoRecorder::stop_locked(const std::unique_lock<std::mutex> &) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_path_.empty()) {
            desired_.store(false);
            return;
        }
    }
    target_->stop_recording();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        spdlog::info("SVO recording stopped: {}", current_path_.string());
        current_path_.clear();
        frame_index_ = -1;
    }
    desired_.store(false);
}

bool SvoRecorder::is_recording() const {
    if (!desired_.load()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return !current_path_.empty();
}

std::string SvoRecorder::current_path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_path_.string();
}

SvoRecorder::FrameRef SvoRecorder::on_frame_grabbed() {
    std::filesystem::path active_path;
    FrameRef frame;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_path = current_path_;
        if (desired_.load() && !active_path.empty()) {
            frame.path = active_path.string();
            frame.index = ++frame_index_;
        }
    }
    if (!desired_.load() || active_path.empty()) return frame;

    capture_thread_frames_since_size_check_++;
    if (capture_thread_frames_since_size_check_ < kSizeCheckIntervalFrames) return frame;
    capture_thread_frames_since_size_check_ = 0;

    std::error_code ec;
    auto file_size = std::filesystem::file_size(active_path, ec);
    if (ec || file_size < max_size_bytes_) return frame;

    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_, std::try_to_lock);
    if (!lifecycle.owns_lock()) return frame;
    if (!desired_.load() || current_path() != active_path.string()) return frame;
    stop_locked(lifecycle);
    start_locked(lifecycle);
    return frame;
}

std::string SvoRecorder::generate_filename() const {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream stamp;
    stamp << std::put_time(&tm, "%Y-%m-%dT%H-%M-%S");
    return unused_svo_path(holding_dir_, stamp.str()).string();
}

void SvoRecorder::enforce_holding_dir_size() {
    std::error_code ec;
    if (!std::filesystem::exists(holding_dir_, ec)) return;

    std::vector<std::filesystem::path> svo_files;
    for (const auto &entry : std::filesystem::directory_iterator(holding_dir_, ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".svo2") {
            svo_files.push_back(entry.path());
        }
    }

    std::sort(svo_files.begin(), svo_files.end(), [](const auto &a, const auto &b) {
        return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
    });

    uintmax_t total_size = 0;
    for (const auto &f : svo_files) {
        total_size += std::filesystem::file_size(f, ec);
    }

    for (const auto &f : svo_files) {
        if (total_size <= holding_dir_max_size_bytes_) break;
        uintmax_t file_size = std::filesystem::file_size(f, ec);
        if (std::filesystem::remove(f, ec)) {
            spdlog::info("Deleted oldest SVO to free space: {}", f.string());
            total_size -= file_size;
        } else {
            spdlog::error("Failed to delete SVO: {}", f.string());
        }
    }
}

}  // namespace auto_battlebot
