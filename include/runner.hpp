#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
#include "target_selector/target_selector_interface.hpp"
#include "time/clock_interface.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {

/** What the Runner shares with remote clients. Every member may be null except `commands`. */
struct RunnerRemote {
    std::shared_ptr<remote::CommandQueue> commands = std::make_shared<remote::CommandQueue>();
    std::shared_ptr<remote::StatusBus> status;
    std::shared_ptr<HostServices> host;
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
           std::shared_ptr<UIState> ui_state = nullptr,
           std::shared_ptr<McapRecorder> mcap_recorder = nullptr,
           std::shared_ptr<ClockInterface> clock = nullptr, RunnerRemote remote = {});

    void initialize();
    void initialize_field(const CameraData &camera_data);
    int run();
    bool tick();

    // Called from the SIGINT/SIGTERM handler, so it only sets a flag the loop polls.
    void request_quit() override { quit_requested_.store(true); }

   private:
    // Independent of ui_state_, which is null whenever the UI is disabled.
    std::atomic<bool> quit_requested_{false};

    /** Drains the command queue and runs each command. Sets `should_reinit_field` when one asked
     *  for a field reinit. Returns false when one stopped the loop (reboot or power off). */
    bool handle_commands(bool &should_reinit_field);
    void set_opponent_count(int count);
    void set_autonomy(bool enabled);
    void set_recording(bool enabled) const;
    std::string select_profile(const std::string &name);
    void run_system_action(SystemAction action);
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
    std::shared_ptr<UIState> ui_state_;
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
    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field_description_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::shared_ptr<HealthLogger> health_logger_;
    std::chrono::steady_clock::time_point start_time_;

    void publish_system_status(bool camera_ok, double loop_rate_hz) const;
    void publish_tracks(const RobotDescriptionsStamped &robots,
                        const FieldDescription &field) const;
    void stop_recordings_for_shutdown() const;
    bool recover_camera_after_failure();
    void set_ui_debug_image_from_camera(const CameraData &camera_data) const;
    bool handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz);
    double elapsed_ms();
};
}  // namespace auto_battlebot
