#pragma once

#include <array>
#include <span>
#include <vector>

#include "enums/hazard_kind.hpp"
#include "rl_policy/observation_builder_interface.hpp"

namespace auto_battlebot::rl_policy {

/**
 * Observation spec v1, the C++ twin of auto_battlebot/rl/obs_spec.py (`build_v1`). 138 floats in
 * our body frame (x forward, y left) at our axle centre; the field frame's origin is the field
 * centre. Blocks, in order:
 *
 *   our robot         6   forward speed, yaw rate, measurement age, stale flag, mode one-hot
 *   history          32   last 8 policy steps, newest first: actions, then measured speed and
 *                         yaw rate
 *   target opponent  16   position, distance, bearing, relative heading, velocity, yaw rate,
 *                         three sigmas, half-diagonal, measurement age, stale flag
 *   range rays       48   24 rays, solid and hole channel each
 *   nearest hazards  36   4 discs by edge distance
 *
 * obs_spec.py holds the scales and caps. Any change here changes the spec: bump the version
 * there, add a new builder here, and regenerate tests/rl_policy/rl_policy_golden_data.hpp with
 * playground/rl/make_rl_golden_fixture.py.
 */
namespace obs_v1 {
inline constexpr int kSpecVersion = 1;
inline constexpr int kSize = 138;
inline constexpr int kHistory = 8;
inline constexpr int kNumRays = 24;
inline constexpr double kRayRangeM = 1.5;
inline constexpr int kMaxHazards = 4;
inline constexpr double kSpeedScale = 2.5;
inline constexpr double kYawRateScale = 15.0;
inline constexpr double kAgeScaleS = 0.5;
inline constexpr double kPositionScaleM = 2.0;
inline constexpr double kPositionSigmaScaleM = 0.1;
inline constexpr double kHeadingSigmaScaleRad = 1.5707963267948966;  // pi / 2
inline constexpr double kFootprintScaleM = 0.2;
inline constexpr double kHazardRadiusScaleM = 0.3;
/** The network input tensor's name in the bundle manifest. */
inline constexpr const char *kTensorName = "obs.vector";
}  // namespace obs_v1

/** One hazard disc, field frame. `radius` is the raw object radius, never inflated. */
struct ObservationHazardV1 {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    HazardKind kind = HazardKind::HOLE;
    double vx = 0.0;
    double vy = 0.0;
    bool present = true;
};

/** One world's inputs, field frame unless noted: `ObsInputs` in obs_spec.py with the batch
 * dimension dropped. Infinite sigmas mean unknown and are allowed. */
struct ObservationInputsV1 {
    double our_x = 0.0;
    double our_y = 0.0;
    double our_yaw = 0.0;
    /** m/s along our heading. */
    double our_speed = 0.0;
    double our_yaw_rate = 0.0;
    double our_age_s = 0.0;
    bool our_stale = false;
    bool run_away = false;
    /** Newest first: linear, angular per step. */
    std::array<double, 2 * obs_v1::kHistory> hist_action{};
    std::array<double, obs_v1::kHistory> hist_speed{};
    std::array<double, obs_v1::kHistory> hist_yaw_rate{};
    bool opp_present = false;
    double opp_x = 0.0;
    double opp_y = 0.0;
    double opp_yaw = 0.0;
    double opp_vx = 0.0;
    double opp_vy = 0.0;
    double opp_yaw_rate = 0.0;
    double opp_position_sigma = 0.0;
    double opp_heading_sigma = 0.0;
    double opp_yaw_rate_sigma = 0.0;
    double opp_half_diagonal = 0.0;
    double opp_age_s = 0.0;
    bool opp_stale = false;
    /** Half the field rectangle's size. */
    double field_half_x = 0.0;
    double field_half_y = 0.0;
    /** Any number of discs; the builder picks the 4 nearest by edge distance. */
    std::span<const ObservationHazardV1> hazards;
};

/** The spec v1 observation. A pure function of `inputs`, computed in double; it does not
 * allocate. */
void build_observation_v1(const ObservationInputsV1 &inputs,
                          std::span<double, obs_v1::kSize> observation);

/**
 * Fill ObservationInputsV1 from the navigation's view of the world. Our forward speed is the
 * filter's field velocity projected on our heading; ages are RobotDescription::measurement_age_s;
 * the opponent's sigmas come from its PoseUncertainty and its half-diagonal from its size. Every
 * FieldHazard becomes a disc of its object_radius. `hazard_storage` backs `inputs.hazards` and is
 * reused across calls.
 */
ObservationInputsV1 observation_inputs_v1(const ObservationContext &context,
                                          std::vector<ObservationHazardV1> &hazard_storage);

/** ObservationBuilderInterface for spec v1: the adapter, then the pure builder, into one
 * float tensor `obs.vector [138]`. */
class ObservationBuilderV1 : public ObservationBuilderInterface {
   public:
    int spec_version() const override { return obs_v1::kSpecVersion; }
    std::vector<TensorSpec> output_specs() const override;
    size_t history_length() const override { return obs_v1::kHistory; }
    void build(const ObservationContext &context, TensorSet &observation) override;

   private:
    std::vector<ObservationHazardV1> hazards_;
    std::array<double, obs_v1::kSize> values_{};
};

}  // namespace auto_battlebot::rl_policy
