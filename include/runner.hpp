#pragma once

#include <miniros/ros.h>

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/target_selection.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "health/config.hpp"
#include "health/health_logger.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "navigation/navigation_interface.hpp"
#include "perception_batch/parallel_model_batch.hpp"
#include "publisher/publisher_interface.hpp"
#include "quittable.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"
#include "robot_descriptions_cache.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "runner_config.hpp"
#include "target_selector/target_selector_interface.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {
// Quittable so SIGINT and SIGTERM can stop the run. The UI manager used to be the only
// quittable, which meant that with ui.enable = false nothing was registered at all and
// both signals became no-ops: the process could then only be killed with SIGKILL.
class Runner : public Quittable {
   public:
    using SystemActionCallback = std::function<void(UISystemAction)>;
    using ProfileSelectCallback = std::function<void(const std::string &)>;

    Runner(const RunnerConfiguration &runner_config, std::shared_ptr<RgbdCameraInterface> camera,
           std::shared_ptr<HealthLogger> health_logger,
           std::shared_ptr<MaskModelInterface> field_model,
           std::shared_ptr<RobotBlobModelInterface> robot_mask_model,
           std::shared_ptr<FieldFilterInterface> field_filter,
           std::shared_ptr<KeypointModelInterface> keypoint_model,
           std::shared_ptr<ParallelModelBatch> perception_batch,
           std::shared_ptr<RobotFilterInterface> robot_filter,
           std::shared_ptr<TargetSelectorInterface> target_selector,
           std::shared_ptr<NavigationInterface> navigation,
           std::shared_ptr<TransmitterInterface> transmitter,
           std::shared_ptr<PublisherInterface> publisher,
           SystemActionCallback system_action_callback,
           ProfileSelectCallback profile_select_callback = nullptr,
           std::shared_ptr<UIState> ui_state = nullptr,
           std::shared_ptr<McapRecorder> mcap_recorder = nullptr,
           std::shared_ptr<ClockInterface> clock = nullptr);

    void initialize();
    void initialize_field(const CameraData &camera_data);
    int run();
    bool tick();

    // Called from the SIGINT/SIGTERM handler, so it only sets a flag the loop polls.
    void request_quit() override { quit_requested_.store(true); }

   private:
    // Independent of ui_state_, which is null whenever the UI is disabled.
    std::atomic<bool> quit_requested_{false};

    RunnerConfiguration runner_config_;
    std::shared_ptr<RgbdCameraInterface> camera_;
    std::shared_ptr<MaskModelInterface> field_model_;
    std::shared_ptr<RobotBlobModelInterface> robot_mask_model_;
    std::shared_ptr<FieldFilterInterface> field_filter_;
    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    // Runs keypoint_model_ and robot_mask_model_ in parallel each tick.
    std::shared_ptr<ParallelModelBatch> perception_batch_;
    std::shared_ptr<RobotFilterInterface> robot_filter_;
    std::shared_ptr<TargetSelectorInterface> target_selector_;
    std::shared_ptr<NavigationInterface> navigation_;
    std::shared_ptr<TransmitterInterface> transmitter_;
    std::shared_ptr<PublisherInterface> publisher_;
    std::shared_ptr<UIState> ui_state_;
    std::shared_ptr<McapRecorder> mcap_recorder_;
    std::shared_ptr<ClockInterface> clock_;
    SystemActionCallback system_action_callback_;
    ProfileSelectCallback profile_select_callback_;

    int runtime_opponent_count_;
    bool robot_filter_reinit_pending_;
    TargetSelection previous_selected_target_;
    RobotDescriptionsCache robot_descriptions_cache_;

    bool initialized_;
    bool autonomy_enabled_;
    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field_description_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::shared_ptr<HealthLogger> health_logger_;
    std::chrono::steady_clock::time_point start_time_;

    void publish_system_status(bool camera_ok, double loop_rate_hz) const;
    void stop_recordings_for_shutdown() const;
    void handle_opponent_count_request();
    void handle_autonomy_toggle_request();
    void handle_recording_toggle_request() const;
    bool handle_system_action_request();
    void handle_profile_switch_request();
    bool handle_ui_requests(bool &should_reinit_field);
    bool recover_camera_after_failure();
    void set_ui_debug_image_from_camera(const CameraData &camera_data) const;
    bool handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz);
    TargetSelection resolve_target(const RobotDescriptionsStamped &robots,
                                   const FieldDescription &field_description);
    double elapsed_ms();
};
}  // namespace auto_battlebot
