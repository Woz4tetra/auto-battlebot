#pragma once

#include <limits>

#include "data_structures/field.hpp"
#include "data_structures/pose.hpp"
#include "data_structures/velocity.hpp"
#include "hazards/hazard_geometry.hpp"

namespace auto_battlebot {

struct HazardAvoidanceSettings {
    bool tangent_enable = true;
    int tangent_max_iterations = 3;
    double side_release_m = 0.05;
    double waypoint_clearance_m = 0.06;
    double prediction_horizon_s = 0.25;

    double reverse_distance = 0.12;
    double heading_threshold = 1.047;
    double reverse_min_speed_mps = 1.5;

    bool barrier_enable = true;
    double max_linear_speed_fwd = 0.0;
    double max_linear_speed_rev = 0.0;
    double barrier_horizon_s = 0.0;
    double barrier_delay_s = 0.0;
    double excess_gain = 6.5;
};

struct HazardBarrierResult {
    bool engaged = false;
    bool braking = false;
    double approach_speed_bound_mps = std::numeric_limits<double>::infinity();
};

class HazardAvoidance {
   public:
    explicit HazardAvoidance(HazardAvoidanceSettings settings);

    void reset();

    Pose2D steer_around(const Pose2D &our_pose, const Pose2D &goal, const FieldDescription &field);

    HazardBarrierResult limit_command(const Pose2D &our_pose, const FieldDescription &field,
                                      double v_actual, VelocityCommand &command) const;

    bool apply_reverse(const Pose2D &our_pose, const FieldDescription &field,
                       VelocityCommand &command) const;

    bool last_substituted() const { return last_substituted_; }
    PassSide committed_side() const { return committed_side_; }

   private:
    HazardAvoidanceSettings settings_;
    PassSide committed_side_ = PassSide::NONE;
    size_t latched_hazard_index_ = 0;
    bool last_substituted_ = false;
};

}  // namespace auto_battlebot
