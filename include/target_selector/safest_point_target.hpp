#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "target_selector/config.hpp"
#include "target_selector/nearest_target.hpp"
#include "target_selector/target_selector_interface.hpp"

namespace auto_battlebot {

class SafestPointTarget : public TargetSelectorInterface {
   public:
    explicit SafestPointTarget(const SafestPointTargetConfiguration &config);
    std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                              const FieldDescription &field,
                                              BehaviorMode mode) override;

   private:
    double circle_radius(double x, double y, const FieldDescription &field,
                         const std::vector<Pose2D> &opponents) const;

    std::optional<Pose2D> solve(const FieldDescription &field, const std::vector<Pose2D> &opponents,
                                const std::optional<Pose2D> &our_pose) const;

    NearestTarget attack_selector_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    double retarget_improvement_m_;
    std::optional<Pose2D> held_target_;
};

}  // namespace auto_battlebot
