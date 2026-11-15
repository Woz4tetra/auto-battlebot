#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <sl/Camera.hpp>
#include <string>

namespace auto_battlebot {

class SvoRecorder {
   public:
    SvoRecorder(sl::Camera &camera, std::filesystem::path holding_dir, uint64_t max_size_bytes,
                uint64_t holding_dir_max_size_bytes);

    void set_desired(bool enabled);

    bool desired() const;

    /**
     * @brief Start recording now. The camera must already be open.
     *
     * Creates the holding directory, evicts old files to stay under the cap,
     * then enables SDK recording. Returns false on SDK failure.
     */
    bool start();

    void stop();

    bool is_recording() const;

    std::string current_path() const;

    struct FrameRef {
        std::string path;
        int64_t index = -1;
    };

    /**
     * @brief Called once per successful grab; rolls the file over past the size cap.
     *
     * Returns where the frame that was just grabbed landed. The return value is captured
     * before any rollover, so it names the file the frame actually went into rather than
     * the fresh one the next frame will use.
     */
    FrameRef on_frame_grabbed();

   private:
    std::string generate_filename() const;
    void enforce_holding_dir_size();
    /** Caller holds lifecycle_mutex_. */
    bool start_locked();
    void stop_locked();

    sl::Camera &camera_;
    std::filesystem::path holding_dir_;
    uint64_t max_size_bytes_;
    uint64_t holding_dir_max_size_bytes_;

    std::atomic<bool> desired_{false};
    std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    std::filesystem::path current_path_;
    /** Capture thread only. */
    uint64_t frames_since_size_check_ = 0;
    int64_t frame_index_ = -1;
};

}  // namespace auto_battlebot
