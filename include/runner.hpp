#pragma once

#include <miniros/ros.h>

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "field_model/field_model_interface.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "navigation/navigation_interface.hpp"
#include "publisher/publisher_interface.hpp"
#include "rgbd_camera/rgbd_camera_interface.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "runner_config.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {
class Runner {
   public:
    Runner(const RunnerConfiguration &runner_config, const std::vector<RobotConfig> &robot_configs,
           std::shared_ptr<RgbdCameraInterface> camera,
           std::shared_ptr<FieldModelInterface> field_model,
           std::shared_ptr<FieldFilterInterface> field_filter,
           std::shared_ptr<KeypointModelInterface> keypoint_model,
           std::shared_ptr<RobotFilterInterface> robot_filter,
           std::shared_ptr<NavigationInterface> navigation,
           std::shared_ptr<TransmitterInterface> transmitter,
           std::shared_ptr<PublisherInterface> publisher,
           std::shared_ptr<UIState> ui_state = nullptr);

    void initialize();
    void initialize_field(const CameraData &camera_data);
    int run();
    bool tick();

   private:
    RunnerConfiguration runner_config_;
    std::shared_ptr<RgbdCameraInterface> camera_;
    std::shared_ptr<FieldModelInterface> field_model_;
    std::shared_ptr<FieldFilterInterface> field_filter_;
    std::shared_ptr<KeypointModelInterface> keypoint_model_;
    std::shared_ptr<RobotFilterInterface> robot_filter_;
    std::shared_ptr<NavigationInterface> navigation_;
    std::shared_ptr<TransmitterInterface> transmitter_;
    std::shared_ptr<PublisherInterface> publisher_;
    std::shared_ptr<UIState> ui_state_;

    std::vector<RobotConfig> initial_robot_configs_;
    int runtime_opponent_count_;
    bool robot_filter_reinit_pending_;

    bool initialized_;
    std::shared_ptr<FieldDescriptionWithInlierPoints> initial_field_description_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
    std::chrono::steady_clock::time_point start_time_;

    std::vector<RobotConfig> build_effective_robot_configs() const;
    double elapsed_ms();
};
}  // namespace auto_battlebot
