#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "command_shaper/command_shaper_interface.hpp"
#include "data_structures.hpp"
#include "data_structures/target_selection.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/behavior_mode.hpp"
#include "hazards/hazard_assembler.hpp"
#include "health/performance_monitor.hpp"
#include "navigation/navigation_interface.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "target_selector/target_selector_interface.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

struct ControlMeasurement {
    ModelResultStamped keypoints;
    FieldDescription field_description;
    CameraInfo camera_info;
};

struct ControlOutput {
    bool valid = false;
    RobotDescriptionsStamped robots;
    TargetSelection target;
    VelocityCommand command;
    CommandFeedback command_feedback;
    BehaviorMode behavior_mode = BehaviorMode::ATTACK;
    bool manual_target = false;
    std::vector<FieldHazard> hazards;
};

class ControlLoop {
   public:
    ControlLoop(std::shared_ptr<RobotFilterInterface> robot_filter,
                std::shared_ptr<TargetSelectorInterface> target_selector,
                std::shared_ptr<NavigationInterface> navigation,
                std::shared_ptr<CommandShaperInterface> command_shaper,
                std::shared_ptr<TransmitterInterface> transmitter,
                std::shared_ptr<ClockInterface> clock,
                std::shared_ptr<HazardAssembler> hazard_assembler);

    bool initialize();

    void set_performance_monitor(std::shared_ptr<PerformanceMonitor> monitor);
    void note_camera_frame();

    void pump_input();

    bool take_init_button_press();

    void submit_measurement(ControlMeasurement measurement);

    void run_cycle();
    void run_cycle_at(double now);

    ControlOutput latest_output() const;

    void request_filter_reinit(int opponent_count);
    void set_autonomy_enabled(bool enabled);
    void set_behavior_mode(BehaviorMode mode);
    void set_target_override(std::optional<Pose2D> target);

    TransmitterStatus transmitter_status() const { return transmitter_status_.load(); }
    NavigationVisualization last_visualization() const;

    std::chrono::steady_clock::time_point last_cycle_wall_time() const;

   private:
    void apply_requests_from_other_threads();
    void monitor_cycle(double now, const RobotDescriptionsStamped *robots,
                       const std::optional<StickCommand> &shaped, bool measured);
    void publish_monitor(double now);
    std::shared_ptr<RobotFilterInterface> robot_filter_;
    std::shared_ptr<TargetSelectorInterface> target_selector_;
    std::shared_ptr<NavigationInterface> navigation_;
    std::shared_ptr<CommandShaperInterface> command_shaper_;
    std::shared_ptr<TransmitterInterface> transmitter_;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<HazardAssembler> hazard_assembler_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    TargetSelection previous_selected_target_{};
    CommandFeedback command_feedback_{};
    std::optional<FieldDescription> field_description_;

    mutable std::mutex mutex_;
    std::optional<ControlMeasurement> pending_measurement_;
    std::optional<Pose2D> target_override_;
    ControlOutput output_;
    NavigationVisualization visualization_;

    std::atomic<bool> init_button_latched_{false};
    std::atomic<int> filter_reinit_request_{0};
    std::atomic<bool> autonomy_enabled_{true};
    std::atomic<int> autonomy_applied_{-1};
    std::atomic<TransmitterStatus> transmitter_status_{};
    std::atomic<BehaviorMode> behavior_mode_{BehaviorMode::ATTACK};
    std::atomic<int64_t> last_cycle_us_{0};
    std::atomic<int> opponent_count_{0};
    std::atomic<bool> camera_frame_seen_{false};

    std::shared_ptr<PerformanceMonitor> monitor_;
    std::shared_ptr<DiagnosticsModuleLogger> monitor_logger_;
    std::optional<double> next_evaluate_s_;
    std::optional<double> next_publish_s_;
    uint64_t last_commands_sent_ = 0;
    std::vector<PerformanceCheck> published_warnings_;

    TargetSelection resolve_target(const RobotDescriptionsStamped &robots,
                                   const FieldDescription &field_description, BehaviorMode mode,
                                   const std::optional<Pose2D> &target_override);
};

}  // namespace auto_battlebot
