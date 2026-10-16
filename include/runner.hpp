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
#include "perception_batch/parallel_model_batch.hpp"
#include "publisher/publisher_interface.hpp"
#include "quittable.hpp"
#include "remote/command_queue.hpp"
#include "remote/status_bus.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"
#include "robot_descriptions_cache.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "runner_config.hpp"
#include "serial_worker.hpp"
#include "target_selector/target_selector_interface.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

/** What the Runner shares with remote clients. Every member may be null except `commands`. */
struct RunnerRemote {
    std::shared_ptr<remote::CommandQueue> commands = std::make_shared<remote::CommandQueue>();
    std::shared_ptr<remote::StatusBus> status;
    std::shared_ptr<HostServices> host;
    /** Saved camera profiles. Null: only DEFAULT is offered. */
    std::shared_ptr<CameraProfileStore> camera_profiles;
    /** Republished on /status/app; the Runner fills in nothing, main.cpp builds it. */
    remote::AppInfoMessage app_info;
};

// Quittable so SIGINT and SIGTERM can stop the run. The UI manager used to be the only
// quittable, which meant that with ui.enable = false nothing was registered at all and
// both signals became no-ops: the process could then only be killed with SIGKILL.
class Runner : public Quittable {
   public:
    using SystemActionCallback = std::function<void(SystemAction)>;
    using ProfileSelectCallback = std::function<void(const std::string &)>;

    Runner(const RunnerConfiguration &runner_config, std::shared_ptr<RgbdCameraInterface> camera,
           std::shared_ptr<HealthLogger> health_logger,
           std::shared_ptr<MaskModelInterface> field_model,
           std::shared_ptr<RobotBlobModelInterface> robot_mask_model,
           std::shared_ptr<FieldFilterInterface> field_filter,
           std::shared_ptr<KeypointModelInterface> keypoint_model,
           std::shared_ptr<KeypointHeightGate> height_gate,
           std::shared_ptr<StaticDetectionGate> static_gate,
           std::shared_ptr<ParallelModelBatch> perception_batch,
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
    /** The /status/system message the last tick built, whether or not a status bus exists. */
    const remote::SystemStatusMessage &system_status() const { return last_status_; }
    /** /status/camera_settings as it would publish now. */
    remote::CameraSettingsMessage camera_settings_status() const;

    // Called from the SIGINT/SIGTERM handler, so it only sets a flag the loop polls.
    ~Runner() override;
    /** Also cancels a camera re-open in progress. Called from the signal handler. */
    void request_quit() override;

   private:
    std::atomic<bool> quit_requested_{false};

    /** Drains the command queue and runs each command. Sets `should_reinit_field` when one asked
     *  for a field reinit. Returns false when one stopped the loop (reboot or power off). */
    bool handle_commands(bool &should_reinit_field);
    void set_opponent_count(int count);
    void set_autonomy(bool enabled);
    void set_behavior_mode(BehaviorMode mode);
    void set_auto_tracking(bool enabled);
    /** Returns the ack message and sets `accepted` false on a refusal. */
    std::string set_manual_target(const remote::SetManualTargetCommand &command, bool &accepted);
    /** Posts the toggle to recording_worker_ and returns. */
    void set_recording(bool enabled) const;
    /** The toggle itself. `from_worker` is false only during shutdown, which runs on the tick's
     *  own thread once the worker has stopped, so there is no tick in flight to wait for. */
    void apply_recording(bool enabled, bool from_worker) const;
    void wait_for_frames_in_flight() const;
    /** Counted at the top of tick(), so the recording worker can tell when frames taken before
     *  a toggle have been published. */
    std::atomic<uint64_t> ticks_started_{0};
    std::string select_profile(const std::string &name);
    void run_system_action(SystemAction action);
    /** Each returns the ack message and sets `accepted` false on a refusal. */
    std::string set_camera_settings(const CameraSettings &settings, bool &accepted);
    std::string run_camera_profile(CameraProfileAction action, const std::string &name,
                                   bool &accepted);
    /** /status/camera_settings: right after a camera command, otherwise twice a second so the
     *  auto-exposure read-back stays current. */
    void publish_camera_settings(bool now);
    /** The app always starts on DEFAULT; nothing remembers the last profile across restarts. */
    std::string active_camera_profile_ = CameraProfileStore::kDefaultName;
    /** What the dashboard last asked the camera for; nullopt while on DEFAULT. SAVE stores this
     *  rather than the read-back, which can trail a request by a frame. */
    std::optional<CameraSettings> requested_camera_settings_;
    bool camera_settings_modified_ = false;
    std::chrono::steady_clock::time_point last_camera_settings_publish_{};
    void ack(std::string_view topic, bool accepted, std::string message = {});

    RunnerConfiguration runner_config_;
    std::shared_ptr<RgbdCameraInterface> camera_;
    std::shared_ptr<MaskModelInterface> field_model_;
    std::shared_ptr<RobotBlobModelInterface> robot_mask_model_;
    std::shared_ptr<FieldFilterInterface> field_filter_;
    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    // Depth-based height gate, applied to both keypoint streams; static-position gate, applied to
    // robot blobs only. Both sit between perception and the control loop.
    std::shared_ptr<KeypointHeightGate> height_gate_;
    std::shared_ptr<StaticDetectionGate> static_gate_;
    // Runs keypoint_model_ and robot_mask_model_ in parallel each tick.
    std::shared_ptr<ParallelModelBatch> perception_batch_;
    /** Owns the filter/target/navigation/transmit half. The Runner reaches the transmitter through
     *  it once running, since a threaded driver owns it on another thread. */
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
    /** Frames left in the current field-init attempt. See handle_tick. */
    int field_init_attempts_remaining_ = 0;
    /** One second at 60 Hz. Long enough for a fiducial board to accumulate, short enough that a
     *  request with no board in frame stops running the mask model. */
    static constexpr int kFieldInitAttempts = 60;
    bool autonomy_enabled_;
    BehaviorMode behavior_mode_;
    bool auto_tracking_;
    /** The last tick's field and camera, for projecting a camera-view tap onto the floor. Empty
     *  until the field is initialized. */
    std::optional<FieldDescription> projection_field_;
    CameraInfo projection_camera_info_;
    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field_description_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::shared_ptr<HealthLogger> health_logger_;
    /** Start of the previous tick, for the loop rate. */
    std::chrono::steady_clock::time_point last_tick_time_;
    /** When the Runner was built, for /status/system uptime. */
    std::chrono::steady_clock::time_point app_start_time_;
    /** When the transmitter's autonomy switch last went on; empty while it is off. */
    std::optional<std::chrono::steady_clock::time_point> autonomy_switch_since_;

    /** Builds /status/system, keeps it for system_status(), and publishes it. */
    void publish_system_status(bool camera_ok, double loop_rate_hz);
    remote::SystemStatusMessage last_status_;
    void publish_tracks(const RobotDescriptionsStamped &robots, const FieldDescription &field,
                        const CameraInfo &camera_info, const TargetSelection &target,
                        bool manual_target) const;
    /** Starts or clears the autonomy-switch timer from the transmitter, and logs both timers. */
    void update_timers();
    double uptime_s() const;
    std::optional<double> autonomy_on_s() const;
    void stop_recordings_for_shutdown() const;
    /** Turns recording off and on when the MCAP passes max_size_gb, or when the camera's
     *  encoder stopped taking frames. */
    void roll_over_recording_if_full();
    std::chrono::steady_clock::time_point last_size_check_{};
    bool recover_camera_after_failure();
    bool handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz);
    double elapsed_ms();

    /** Held by apply_recording and by camera reinitialization, which both start and stop the
     *  camera's encoder or SVO recorder. */
    mutable std::mutex recording_mutex_;
    /** A rollover is posted and not yet done. */
    std::atomic<bool> rollover_pending_{false};
    /** The control loop has missed its watchdog, and since when. Loop thread only. */
    bool control_loop_stalled_ = false;
    std::chrono::steady_clock::time_point control_loop_stalled_since_{};
    /** Runs recording toggles and rollovers off the tick. Last member, so it is joined before
     *  anything its jobs use is destroyed. */
    std::unique_ptr<SerialWorker> recording_worker_ = std::make_unique<SerialWorker>();
};
}  // namespace auto_battlebot
