#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "config/camera_profile_store.hpp"
#include "control_loop/control_loop_interface.hpp"
#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/target_selection.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "enums/system_action.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "health/config.hpp"
#include "health/health_logger.hpp"
#include "host/host_services.hpp"
#include "keypoint_filter/height_gate.hpp"
#include "keypoint_filter/static_gate.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "navigation/navigation_interface.hpp"
#include "publisher/publisher_interface.hpp"
#include "quittable.hpp"
#include "remote/command_queue.hpp"
#include "remote/status_bus.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "runner_config.hpp"
#include "serial_worker.hpp"
#include "target_selector/target_selector_interface.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

struct RunnerRemote {
    std::shared_ptr<remote::CommandQueue> commands = std::make_shared<remote::CommandQueue>();
    std::shared_ptr<remote::StatusBus> status;
    std::shared_ptr<HostServices> host;
    std::shared_ptr<CameraProfileStore> camera_profiles;
    remote::AppInfoMessage app_info;
    remote::ResolvedConfigMessage resolved_config;
};

class Runner : public Quittable {
   public:
    using SystemActionCallback = std::function<void(SystemAction)>;
    using ProfileSelectCallback = std::function<void(const std::string &)>;

    Runner(const RunnerConfiguration &runner_config, std::shared_ptr<RgbdCameraInterface> camera,
           std::shared_ptr<HealthLogger> health_logger,
           std::shared_ptr<MaskModelInterface> field_model,
           std::shared_ptr<FieldFilterInterface> field_filter,
           std::shared_ptr<KeypointModelInterface> keypoint_model,
           std::shared_ptr<KeypointHeightGate> height_gate,
           std::shared_ptr<StaticDetectionGate> static_gate,
           std::shared_ptr<ControlLoopInterface> control_loop,
           std::shared_ptr<PublisherInterface> publisher,
           SystemActionCallback system_action_callback,
           ProfileSelectCallback profile_select_callback = nullptr,
           std::shared_ptr<McapRecorder> mcap_recorder = nullptr,
           std::shared_ptr<ClockInterface> clock = nullptr, RunnerRemote remote = {});

    void initialize();
    void initialize_field(const CameraData &camera_data);
    int run();
    bool tick();
    const remote::SystemStatusMessage &system_status() const { return last_status_; }
    remote::CameraSettingsMessage camera_settings_status() const;

    ~Runner() override;
    void quit_from_signal_handler() override;

   private:
    std::atomic<bool> quit_requested_{false};

    bool handle_commands(bool &should_reinit_field);
    void set_opponent_count(int count);
    void set_autonomy(bool enabled);
    void set_behavior_mode(BehaviorMode mode);
    void set_auto_tracking(bool enabled);
    std::string set_manual_target(const remote::SetManualTargetCommand &command, bool &accepted);
    void set_recording(bool enabled) const;
    void apply_recording(bool enabled) const;
    std::string select_profile(const std::string &name);
    void run_system_action(SystemAction action);
    std::string set_camera_settings(const CameraSettings &settings, bool &accepted);
    std::string run_camera_profile(CameraProfileAction action, const std::string &name,
                                   bool &accepted);
    void publish_camera_settings(bool now);
    std::string active_camera_profile_ = CameraProfileStore::kDefaultName;
    std::optional<CameraSettings> requested_camera_settings_;
    bool camera_settings_modified_ = false;
    std::chrono::steady_clock::time_point last_camera_settings_publish_{};
    void ack(std::string_view topic, bool accepted, std::string message = {});

    RunnerConfiguration runner_config_;
    std::shared_ptr<RgbdCameraInterface> camera_;
    std::shared_ptr<MaskModelInterface> field_model_;
    std::shared_ptr<FieldFilterInterface> field_filter_;
    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    std::shared_ptr<KeypointHeightGate> height_gate_;
    std::shared_ptr<StaticDetectionGate> static_gate_;
    std::shared_ptr<ControlLoopInterface> control_loop_;
    std::shared_ptr<PublisherInterface> publisher_;
    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::shared_ptr<ClockInterface> clock_;
    SystemActionCallback system_action_callback_;
    ProfileSelectCallback profile_select_callback_;
    RunnerRemote remote_;
    remote::CommandLog command_log_;
    int64_t ack_seq_ = 0;

    int runtime_opponent_count_;

    bool initialized_;
    int field_init_attempts_remaining_ = 0;
    static constexpr int kFieldInitAttempts = 60;
    bool autonomy_enabled_;
    BehaviorMode behavior_mode_;
    bool auto_tracking_;
    std::optional<FieldDescription> projection_field_;
    CameraInfo projection_camera_info_;
    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field_description_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::shared_ptr<HealthLogger> health_logger_;
    std::chrono::steady_clock::time_point prev_tick_start_;
    std::chrono::steady_clock::time_point constructed_at_;
    std::optional<std::chrono::steady_clock::time_point> autonomy_switch_on_since_;

    void publish_system_status(bool camera_ok, double loop_rate_hz);
    remote::SystemStatusMessage last_status_;
    void publish_tracks(const RobotDescriptionsStamped &robots, const FieldDescription &field,
                        const CameraInfo &camera_info, const TargetSelection &target,
                        bool manual_target) const;
    void update_timers();
    double uptime_s() const;
    std::optional<double> autonomy_on_s() const;
    void stop_recordings_for_shutdown() const;
    void roll_over_recording_if_full();
    std::chrono::steady_clock::time_point last_size_check_{};
    bool recover_camera_after_failure();
    bool handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz);
    double elapsed_ms();

    mutable std::mutex recording_mutex_;
    std::atomic<bool> rollover_pending_{false};
    bool loop_thread_control_loop_stalled_ = false;
    std::chrono::steady_clock::time_point loop_thread_control_loop_stalled_since_{};
    std::unique_ptr<SerialWorker> recording_worker_ = std::make_unique<SerialWorker>();
};
}  // namespace auto_battlebot
