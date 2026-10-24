#pragma once

#include <memory>
#include <optional>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "navigation/config.hpp"
#include "navigation/navigation_interface.hpp"
#include "rl_policy/action_history.hpp"
#include "rl_policy/observation_builder_interface.hpp"
#include "rl_policy/policy_interface.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

/**
 * Drives with a policy from a model bundle (docs/plans/rl_policy_navigation_plan.md section 12).
 *
 * This class finds our robot and the target opponent, schedules policy steps and logs. The
 * bundle decides everything else: the policy kind, the network runtime, the decoder and the
 * observation spec. Every policy_period_ticks control ticks it builds the observation, runs the
 * policy and stores the action; every tick it returns the stored action. Nothing edits the action
 * on its way to the transmitter: there is no HazardAvoidance here, and hazards reach the policy
 * only as observations.
 *
 * initialize() loads and checks the bundle the first time it runs and throws naming what failed:
 * opponent_mode must be KALMAN, the manifest must parse with known values, a builder must exist
 * for its obs_spec_version and make the tensors the network reads, and the self-test must pass.
 * Every call resets the policy, the history and the schedule.
 *
 * Once our robot has gone unmeasured for longer than max_our_unmeasured_s it commands zero, the
 * same as when our robot is missing. The coasted track keeps reporting the last yaw rate, so the
 * policy otherwise holds whatever it was doing when the keypoints dropped, usually a full-rate
 * spin, and the motion blur from that spin keeps the keypoint model from reacquiring the robot.
 * Stopping lets it reacquire, and the next policy step runs on the first measured tick.
 */
class RlPolicyNavigation : public NavigationInterface {
   public:
    RlPolicyNavigation(const RlPolicyNavigationConfiguration &config,
                       std::shared_ptr<ClockInterface> clock);

    bool initialize() override;

    VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                           const TargetSelection &target) override;

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

   private:
    /** The bundle load and every startup check. Throws. */
    void load();

    /** ATTACK: the opponent track nearest the selected target. RUN_AWAY: the opponent track
     * nearest our robot. Null when there is no opponent track. */
    static const RobotDescription *find_target_opponent(const RobotDescriptionsStamped &robots,
                                                        const RobotDescription &our_robot,
                                                        const TargetSelection &target);

    void log_step(const RobotDescriptionsStamped &robots, const RobotDescription &our_robot,
                  const RobotDescription *opponent, const FieldDescription &field,
                  double inference_us);

    RlPolicyNavigationConfiguration config_;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    std::unique_ptr<rl_policy::PolicyInterface> policy_;
    std::unique_ptr<rl_policy::ObservationBuilderInterface> builder_;
    rl_policy::TensorSet observation_;
    std::optional<rl_policy::ActionHistory> history_;
    bool loaded_ = false;
    std::string run_id_;
    std::string selftest_summary_;
    int policy_period_ticks_ = 1;

    /** Control ticks until the next policy step; 0 steps on this tick. */
    int ticks_until_step_ = 0;
    rl_policy::ActionCommand action_;
    /** Frame of the opponent the policy last saw as its target; EMPTY for none. A change resets
     * the policy. */
    FrameId target_frame_id_ = FrameId::EMPTY;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
