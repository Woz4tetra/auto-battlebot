#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
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
    /** Radius of the largest circle centered at (x, y) that clears the walls, every opponent,
     *  and every hazard. Used to re-measure the held target each cycle -- with hazards in the
     *  same objective the solver optimizes, so a held target that a moving hazard has since
     *  covered loses to a fresh candidate instead of being defended by hysteresis. */
    double circle_radius(double x, double y, const FieldDescription &field,
                         const std::vector<Pose2D> &opponents) const;

    /** Center of the largest empty circle via the exact constraint-triple solver, the
     *  winner of the experiment in
     *  docs/experiments/control_improvement/run_away_solver_report.md. Ties on the
     *  no-opponent plateau resolve toward our_pose so we do not cross the field for
     *  nothing; without our pose the search still runs and only the tie-break degrades. */
    std::optional<Pose2D> solve(const FieldDescription &field, const std::vector<Pose2D> &opponents,
                                const std::optional<Pose2D> &our_pose) const;

    NearestTarget attack_selector_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    double retarget_improvement_m_;
    std::optional<Pose2D> held_target_;
};

}  // namespace auto_battlebot
