#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "data_structures.hpp"
#include "data_structures/target_selection.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/behavior_mode.hpp"
#include "hazards/hazard_assembler.hpp"
#include "navigation/navigation_interface.hpp"
#include "robot_descriptions_cache.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "target_selector/target_selector_interface.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

struct ControlMeasurement {
    ModelResultStamped keypoints;
    ModelResultStamped robot_blob_keypoints;
    FieldDescription field_description;
    CameraInfo camera_info;
};

struct ControlOutput {
    bool valid = false;
    RobotDescriptionsStamped robots;
    RobotDescriptionsStamped cached_robots;
    bool using_previous = false;
    TargetSelection target;
    VelocityCommand command;
    /** What the transmitter read back for this cycle, in normalized stick units. On OpenTX this
     *  is the decoded position of the drive channels, so it reflects the mixed stick plus trainer
     *  output rather than what navigation asked for. */
    CommandFeedback command_feedback;
    bool our_blob_present_no_keypoint = false;
    BehaviorMode behavior_mode = BehaviorMode::ATTACK;
    /** `target` is the dashboard's held point rather than the selector's pick. */
    bool manual_target = false;
    std::vector<FieldHazard> hazards;
};

class ControlLoop {
   public:
    ControlLoop(std::shared_ptr<RobotFilterInterface> robot_filter,
                std::shared_ptr<TargetSelectorInterface> target_selector,
                std::shared_ptr<NavigationInterface> navigation,
                std::shared_ptr<TransmitterInterface> transmitter,
                std::shared_ptr<ClockInterface> clock,
                std::shared_ptr<HazardAssembler> hazard_assembler, double robot_cache_max_age_s);

    /**
     * Brings up the transmitter. Called by ControlLoopInterface::start() before any driver can
     * cycle, which is the ordering that keeps a send() from preceding transmitter initialization.
     */
    bool initialize();

    /**
     * Read transmitter input and latch an init-button edge.
     *
     * Kept separate from run_cycle() because the Runner needs the init-button edge before it calls
     * camera_->get(), which is where field reinitialization is triggered. Stepped drivers call
     * this inline from the Runner's thread, before the camera grab; the threaded driver calls it
     * itself at the top of each cycle and the Runner's call is a no-op.
     */
    void pump_input();

    bool take_init_button_press();

    /** Hand over one perception result. The next cycle corrects with it; later cycles predict
     *  only. Replaces any measurement not yet consumed, so a slow control loop never folds a
     *  backlog in all at once. */
    void submit_measurement(ControlMeasurement measurement);

    void run_cycle();

    ControlOutput latest_output() const;

    void request_filter_reinit(int opponent_count);
    void set_autonomy_enabled(bool enabled);
    void set_behavior_mode(BehaviorMode mode);
    /** A field point navigation steers to in place of the target selector's pick; nullopt hands
     *  targeting back to the selector. */
    void set_target_override(std::optional<Pose2D> target);

    TransmitterStatus transmitter_status() const { return transmitter_status_.load(); }
    NavigationVisualization last_visualization() const;

    /** Wall-clock time of the last completed cycle, for the threaded driver's watchdog. */
    std::chrono::steady_clock::time_point last_cycle_time() const;

   private:
    std::shared_ptr<RobotFilterInterface> robot_filter_;
    std::shared_ptr<TargetSelectorInterface> target_selector_;
    std::shared_ptr<NavigationInterface> navigation_;
    std::shared_ptr<TransmitterInterface> transmitter_;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<HazardAssembler> hazard_assembler_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    RobotDescriptionsCache robot_descriptions_cache_;
    TargetSelection previous_selected_target_{};
    CommandFeedback command_feedback_{};
    std::optional<FieldDescription> field_description_;

    mutable std::mutex mutex_;
    std::optional<ControlMeasurement> pending_measurement_;
    ControlOutput output_;
    NavigationVisualization visualization_;

    std::atomic<bool> init_button_latched_{false};
    std::atomic<int> filter_reinit_request_{0};
    std::atomic<bool> autonomy_enabled_{true};
    /** -1 until the first cycle applies it, so the initial state always reaches the transmitter. */
    std::atomic<int> autonomy_applied_{-1};
    std::atomic<TransmitterStatus> transmitter_status_{};
    std::atomic<BehaviorMode> behavior_mode_{BehaviorMode::ATTACK};
    /** Guarded by mutex_. */
    std::optional<Pose2D> target_override_;
    std::atomic<int64_t> last_cycle_us_{0};

    TargetSelection resolve_target(const RobotDescriptionsStamped &robots,
                                   const FieldDescription &field_description, BehaviorMode mode,
                                   const std::optional<Pose2D> &target_override);
};

}  // namespace auto_battlebot
