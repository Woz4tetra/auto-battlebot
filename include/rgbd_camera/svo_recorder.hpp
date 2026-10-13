#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <sl/Camera.hpp>
#include <string>

namespace auto_battlebot {

/**
 * @brief Manages SVO recording for an open ZED camera.
 *
 * Owns the recording lifecycle: filename generation, enabling/disabling
 * recording on the SDK, evicting old files when the holding directory grows too
 * large, and rolling the active file over once it exceeds the size cap.
 *
 * Threading: set_desired()/start()/stop() are driven from the runner thread;
 * on_frame_grabbed() runs on the capture thread. Both call into the same sl::Camera
 * handle. lifecycle_mutex_ serializes start, stop and the size rollover, so a toggle
 * cannot land between the rollover's stop and its start.
 */
class SvoRecorder {
   public:
    SvoRecorder(sl::Camera &camera, std::filesystem::path holding_dir, uint64_t max_size_bytes,
                uint64_t holding_dir_max_size_bytes);

    /// Set the desired recording state (used before the camera is open).
    void set_desired(bool enabled);

    /// Desired recording state.
    bool desired() const;

    /**
     * @brief Start recording now. The camera must already be open.
     *
     * Creates the holding directory, evicts old files to stay under the cap,
     * then enables SDK recording. Returns false on SDK failure.
     */
    bool start();

    /// Stop recording if active.
    void stop();

    /// True when actively writing a file and recording is desired.
    bool is_recording() const;

    /// Path of the file currently being written, or "" if not recording.
    std::string current_path() const;

    /// Identifies one recorded SVO frame. Empty path and index -1 when recording is off.
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
    /** Held across the SDK calls in start, stop and rollover. Never taken per frame. */
    std::mutex lifecycle_mutex_;
    /** Guards current_path_ and frame_index_; held briefly, never across an SDK call. */
    mutable std::mutex mutex_;
    std::filesystem::path current_path_;
    /** Capture thread only. */
    uint64_t frames_since_size_check_ = 0;
    /// Index of the last frame written to current_path_; -1 before the first frame of a file.
    int64_t frame_index_ = -1;
};

}  // namespace auto_battlebot
