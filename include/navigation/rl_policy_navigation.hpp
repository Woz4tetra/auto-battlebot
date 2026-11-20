#pragma once

#include <memory>
#include <optional>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "navigation/config.hpp"
#include "navigation/hazard_avoidance.hpp"
#include "navigation/navigation_interface.hpp"
#include "rl_policy/action_history.hpp"
#include "rl_policy/observation_builder_interface.hpp"
#include "rl_policy/policy_interface.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {

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
    void load();

    static const RobotDescription *find_target_opponent(const RobotDescriptionsStamped &robots,
                                                        const RobotDescription &our_robot,
                                                        const TargetSelection &target);

    void log_step(double observation_age_s, const RobotDescription &our_robot,
                  const RobotDescription *opponent, const FieldDescription &field,
                  double inference_us);

    RlPolicyNavigationConfiguration config_;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::optional<HazardAvoidance> hazards_;

    std::unique_ptr<rl_policy::PolicyInterface> policy_;
    std::unique_ptr<rl_policy::ObservationBuilderInterface> builder_;
    rl_policy::TensorSet observation_;
    std::optional<rl_policy::ActionHistory> history_;
    bool loaded_ = false;
    std::string run_id_;
    std::string selftest_summary_;
    int policy_period_ticks_ = 1;

    int ticks_until_policy_step_ = 0;
    rl_policy::ActionCommand action_;
    FrameId target_frame_id_ = FrameId::EMPTY;
    NavigationVisualization last_visualization_;
};

}  // namespace auto_battlebot
