#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/keypoint.hpp"
#include "data_structures/robot.hpp"
#include "data_structures/target_selection.hpp"
#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "navigation/navigation_interface.hpp"
#include "ui/battery_options.hpp"

namespace auto_battlebot {
enum class UISystemAction : int {
    NONE = 0,
    QUIT_APP = 1,
    REBOOT_HOST = 2,
    POWEROFF_HOST = 3,
};

/** System status written by Runner, read by UI. UI derives our_robot_seen/opponent_count_seen from
 * robots. loop_met is computed in UI from rolling average. */
struct SystemStatus {
    bool camera_ok = false;
    bool transmitter_connected = false;
    double loop_rate_hz = 0.0;
    bool initialized = false;
    int selected_opponent_count = 1;
    bool autonomy_enabled = true;
    bool svo_recording_enabled = false;
    bool mcap_recording_enabled = false;
    bool recording_enabled = false;
    double jetson_temperature_c = 0.0;  // 0 = not available
    std::string jetson_compute_mode;
};

/**
 * Shared state between Runner and UI thread.
 * Runner writes status and debug image; UI reads and draws.
 * UI writes reinit_requested and opponent_count_requested; Runner reads at tick start.
 */
class UIState {
   public:
    UIState() = default;

    // --- Commands from UI (written by UI, read by Runner) ---
    std::atomic<bool> reinit_requested{false};
    /** 1-3 for set count, -1 or 0 = no change */
    std::atomic<int> opponent_count_requested{-1};
    /** Set by UI when window is closed; Runner should exit. */
    std::atomic<bool> quit_requested{false};
    /** 1 = enable, -1 = disable, 0 = no change */
    std::atomic<int> autonomy_toggle_requested{0};
    /** True = toggle both SVO and MCAP recording. */
    std::atomic<bool> recording_toggle_requested{false};
    /** static_cast<int>(UISystemAction) values; consumed by Runner. */
    std::atomic<int> system_action_requested{static_cast<int>(UISystemAction::NONE)};

    // --- Status from Runner (written by Runner, read by UI) ---
    void set_system_status(const SystemStatus &s);
    void get_system_status(SystemStatus &out) const;

    /** Flattened key-value diagnostics from UIDiagnosticsBackend */
    void set_diagnostics(const std::map<std::string, std::string> &kv);
    void get_diagnostics(std::map<std::string, std::string> &out) const;

    /** Structured diagnostic snapshots (preserves section grouping for UI tables) */
    void set_diagnostic_snapshots(const std::vector<DiagnosticStatusSnapshot> &snapshots);
    void get_diagnostic_snapshots(std::vector<DiagnosticStatusSnapshot> &out) const;

    /** Debug image: raw RGB bytes, row-major. Runner sets; UI draws keypoints and displays. */
    void set_debug_image(int width, int height, int channels, const std::vector<uint8_t> &data);
    void get_debug_image(int &width, int &height, int &channels, std::vector<uint8_t> &data) const;

    /** Robot descriptions (tracked robots). UI derives our_robot_seen, opponent_count_seen from
     * these. */
    void set_robots(const RobotDescriptionsStamped &robots);
    void get_robots(RobotDescriptionsStamped &out) const;

    /** Keypoints from detection. UI draws these on the debug image. */
    void set_keypoints(const KeypointsStamped &keypoints);
    void get_keypoints(KeypointsStamped &out) const;

    /** Last navigation path (field frame). UI may draw for debug. */
    void set_navigation_path(const std::optional<NavigationPathSegment> &path);
    void get_navigation_path(std::optional<NavigationPathSegment> &out) const;
    /** Manual target override in field frame while user is press-holding camera feed. */
    void set_manual_target(const std::optional<TargetSelection> &target);
    std::optional<TargetSelection> get_manual_target() const;
    /** Field description used to project field-frame overlays into camera image. */
    void set_field_description(const std::optional<FieldDescription> &field);
    void get_field_description(std::optional<FieldDescription> &out) const;
    /** Camera intrinsics/distortion used for projection into image pixels. */
    void set_camera_info(const CameraInfo &camera_info);
    void get_camera_info(CameraInfo &out) const;

    /** Window size for UI (from config [ui]; default 1024x600). Set before starting UI thread. */
    void set_window_size(int width, int height);
    void get_window_size(int &width, int &height) const;

    void set_fullscreen(bool fullscreen);
    bool get_fullscreen() const;
    void set_battery_source(const std::string &battery_source);
    std::string get_battery_source() const;
    void set_battery_options(const BatteryOptions &options);
    BatteryOptions get_battery_options() const;

    /** Rolling average window size for loop_rate_hz (number of samples). Set before starting UI
     * thread. */
    void set_rate_avg_window(int window);
    int get_rate_avg_window() const;

    /** Target loop rate (Hz); UI uses this to compute loop_met from averaged rate. Set before
     * starting UI thread. */
    void set_max_loop_rate(double hz);
    double get_max_loop_rate() const;

    /** Fraction of max_loop_rate below which rate is "not met". Set before starting UI thread. */
    void set_rate_fail_threshold(double fraction);
    double get_rate_fail_threshold() const;
    /** Seconds rate must be below threshold before showing error (anti-strobe). Set before starting
     * UI thread. */
    void set_rate_fail_duration_sec(double sec);
    double get_rate_fail_duration_sec() const;

   private:
    mutable std::mutex status_mutex_;
    SystemStatus system_status_;
    std::map<std::string, std::string> diagnostics_;
    std::vector<DiagnosticStatusSnapshot> diagnostic_snapshots_;
    int debug_image_width_ = 0;
    int debug_image_height_ = 0;
    int debug_image_channels_ = 0;
    std::vector<uint8_t> debug_image_data_;
    RobotDescriptionsStamped robots_;
    KeypointsStamped keypoints_;
    std::optional<NavigationPathSegment> navigation_path_;
    std::optional<TargetSelection> manual_target_;
    std::optional<FieldDescription> field_description_;
    CameraInfo camera_info_;
    int window_width_ = 1024;
    int window_height_ = 600;
    bool fullscreen_ = true;
    std::string battery_source_ = "Waveshare UPS";
    BatteryOptions battery_options_;
    int rate_avg_window_ = 10;
    double max_loop_rate_hz_ = 300.0;
    double rate_fail_threshold_ = 0.5;
    double rate_fail_duration_sec_ = 2.0;
};

}  // namespace auto_battlebot
