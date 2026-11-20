#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <sl/Camera.hpp>
#include <string>

namespace auto_battlebot {

std::filesystem::path unused_svo_path(const std::filesystem::path &dir, const std::string &stamp);

class SvoRecordingTarget {
   public:
    virtual ~SvoRecordingTarget() = default;
    virtual bool is_open() const = 0;
    virtual bool start_recording(const std::string &path, std::string &error) = 0;
    virtual void stop_recording() = 0;
};

class ZedSvoRecordingTarget : public SvoRecordingTarget {
   public:
    explicit ZedSvoRecordingTarget(sl::Camera &camera) : camera_(camera) {}
    bool is_open() const override;
    bool start_recording(const std::string &path, std::string &error) override;
    void stop_recording() override;

   private:
    sl::Camera &camera_;
};

class SvoRecorder {
   public:
    SvoRecorder(std::unique_ptr<SvoRecordingTarget> target, std::filesystem::path holding_dir,
                uint64_t max_size_bytes, uint64_t holding_dir_max_size_bytes);
    SvoRecorder(sl::Camera &camera, std::filesystem::path holding_dir, uint64_t max_size_bytes,
                uint64_t holding_dir_max_size_bytes);

    void set_desired(bool enabled);

    bool desired() const;

    bool start();

    void stop();

    bool is_recording() const;

    std::string current_path() const;

    struct FrameRef {
        std::string path;
        int64_t index = -1;
    };

    FrameRef on_frame_grabbed();

   private:
    std::string generate_filename() const;
    void enforce_holding_dir_size();
    bool start_locked(const std::unique_lock<std::mutex> &lifecycle_lock);
    void stop_locked(const std::unique_lock<std::mutex> &lifecycle_lock);

    std::unique_ptr<SvoRecordingTarget> target_;
    std::filesystem::path holding_dir_;
    uint64_t max_size_bytes_;
    uint64_t holding_dir_max_size_bytes_;

    std::atomic<bool> desired_{false};
    std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    std::filesystem::path current_path_;
    uint64_t capture_thread_frames_since_size_check_ = 0;
    int64_t frame_index_ = -1;
};

}  // namespace auto_battlebot
