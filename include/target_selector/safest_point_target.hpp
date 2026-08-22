#pragma once

#include <optional>
#include <vector>

#include "target_selector/config.hpp"
#include "target_selector/nearest_target.hpp"
#include "target_selector/target_selector_interface.hpp"

namespace auto_battlebot {

/** Answers both behavior modes with one selector: delegates to NearestTarget in ATTACK and
 *  drives to the center of the largest opponent-free circle in RUN_AWAY. Keeping attack
 *  delegation here means a config pointed at SafestPointTarget still fights normally when
 *  the transmitter switch stays in ATTACK. */
class SafestPointTarget : public TargetSelectorInterface {
   public:
    explicit SafestPointTarget(const SafestPointTargetConfiguration &config);
    std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                              const FieldDescription &field,
                                              BehaviorMode mode) override;

   private:
    /** Radius of the largest circle centered at (x, y) that clears the walls and every
     *  opponent. Used to re-measure the held target each cycle. */
    double circle_radius(double x, double y, const FieldDescription &field,
                         const std::vector<Pose2D> &opponents) const;

    /** Center of the largest empty circle. The body will be whichever solver the experiment
     *  in docs/experiments/control_improvement/run_away_solver_report.md picks; until then it
     *  returns std::nullopt, so RUN_AWAY holds the previous target via
     *  ControlLoop::resolve_target(). */
    std::optional<Pose2D> solve(const FieldDescription &field,
                                const std::vector<Pose2D> &opponents) const;

    NearestTarget attack_selector_;
    double retarget_improvement_m_;
    std::optional<Pose2D> held_target_;
};

}  // namespace auto_battlebot
