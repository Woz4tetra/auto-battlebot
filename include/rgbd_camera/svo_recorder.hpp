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
 * on_frame_grabbed() runs on the capture thread. Both may call into the same
 * sl::Camera handle, matching the pre-existing usage pattern.
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

    /// Called once per successful grab; rolls the file over past the size cap.
    void on_frame_grabbed();

   private:
    std::string generate_filename() const;
    void enforce_holding_dir_size();

    sl::Camera &camera_;
    std::filesystem::path holding_dir_;
    uint64_t max_size_bytes_;
    uint64_t holding_dir_max_size_bytes_;

    std::atomic<bool> desired_{false};
    mutable std::mutex mutex_;
    std::filesystem::path current_path_;
    uint64_t frames_since_size_check_ = 0;
};

}  // namespace auto_battlebot
