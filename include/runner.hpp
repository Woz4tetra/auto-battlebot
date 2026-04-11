#pragma once

#include <miniros/ros.h>

#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/target_selection.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "navigation/navigation_interface.hpp"
#include "publisher/publisher_interface.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "runner_config.hpp"
#include "target_selector/target_selector_interface.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {
class Runner {
   public:
    using SystemActionCallback = std::function<void(UISystemAction)>;

    Runner(const RunnerConfiguration &runner_config, const std::vector<RobotConfig> &robot_configs,
           std::shared_ptr<RgbdCameraInterface> camera,
           std::shared_ptr<MaskModelInterface> field_model,
           std::shared_ptr<MaskModelInterface> robot_mask_model,
           std::shared_ptr<FieldFilterInterface> field_filter,
           std::shared_ptr<KeypointModelInterface> keypoint_model,
           std::shared_ptr<RobotFilterInterface> robot_filter,
           std::shared_ptr<TargetSelectorInterface> target_selector,
           std::shared_ptr<NavigationInterface> navigation,
           std::shared_ptr<TransmitterInterface> transmitter,
           std::shared_ptr<PublisherInterface> publisher,
           std::shared_ptr<UIState> ui_state = nullptr,
           std::shared_ptr<McapRecorder> mcap_recorder = nullptr,
           SystemActionCallback system_action_callback = nullptr);

    void initialize();
    void initialize_field(const CameraData &camera_data);
    int run();
    bool tick();

   private:
    RunnerConfiguration runner_config_;
    std::shared_ptr<RgbdCameraInterface> camera_;
    std::shared_ptr<MaskModelInterface> field_model_;
    std::shared_ptr<MaskModelInterface> robot_mask_model_;
    std::shared_ptr<FieldFilterInterface> field_filter_;
    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    std::shared_ptr<RobotFilterInterface> robot_filter_;
    std::shared_ptr<TargetSelectorInterface> target_selector_;
    std::shared_ptr<NavigationInterface> navigation_;
    std::shared_ptr<TransmitterInterface> transmitter_;
    std::shared_ptr<PublisherInterface> publisher_;
    std::shared_ptr<UIState> ui_state_;
    std::shared_ptr<McapRecorder> mcap_recorder_;
    SystemActionCallback system_action_callback_;

    std::vector<RobotConfig> initial_robot_configs_;
    int runtime_opponent_count_;
    bool robot_filter_reinit_pending_;
    TargetSelection previous_selected_target_;

    bool initialized_;
    bool autonomy_enabled_;
    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field_description_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::chrono::steady_clock::time_point start_time_;

    std::vector<RobotConfig> build_effective_robot_configs() const;
    void publish_system_status(bool camera_ok, double loop_rate_hz) const;
    void stop_recordings_for_shutdown() const;
    void handle_opponent_count_request();
    void handle_autonomy_toggle_request();
    void handle_recording_toggle_request() const;
    bool handle_system_action_request();
    bool handle_ui_requests(bool &should_reinit_field);
    bool recover_camera_after_failure();
    void set_ui_debug_image_from_camera(const CameraData &camera_data) const;
    bool handle_uninitialized_tick(const CameraData &camera_data, double loop_rate_hz);
    double elapsed_ms();
};
}  // namespace auto_battlebot
