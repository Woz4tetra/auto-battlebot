// ObservationBuilderV1 against auto_battlebot/rl/obs_spec.py (plan gate G3), and the adapter
// that fills its inputs from the navigation's robots, field and target.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "rl_policy/observations/observation_v1.hpp"
#include "rl_policy/rl_policy_golden_data.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot::rl_policy {
namespace {

constexpr double kTolerance = 1e-5;

// Block offsets in the 138 floats.
constexpr int kOur = 0;
constexpr int kHistory = 6;
constexpr int kOpponent = 38;
constexpr int kRays = 54;
constexpr int kHazards = 102;

using Observation = std::array<double, obs_v1::kSize>;

Observation build(const ObservationInputsV1 &inputs) {
    Observation out{};
    build_observation_v1(inputs, out);
    return out;
}

TEST(ObservationV1GoldenTest, MatchesObsSpecPyOnEveryCase) {
    static_assert(rl_golden::kObsSpecVersion == obs_v1::kSpecVersion);
    static_assert(rl_golden::kObsDim == obs_v1::kSize);
    ASSERT_EQ(rl_golden::cases().size(), 24u);
    for (const auto &c : rl_golden::cases()) {
        std::vector<ObservationHazardV1> hazards;
        for (const auto &h : c.hazards) {
            hazards.push_back(ObservationHazardV1{
                h.x, h.y, h.radius, static_cast<HazardKind>(h.kind), h.vx, h.vy, true});
        }
        ObservationInputsV1 in;
        in.our_x = c.our_x;
        in.our_y = c.our_y;
        in.our_yaw = c.our_yaw;
        in.our_speed = c.our_speed;
        in.our_yaw_rate = c.our_yaw_rate;
        in.our_age_s = c.our_age_s;
        in.our_stale = c.our_stale;
        in.run_away = c.run_away;
        in.hist_action = c.hist_action;
        in.hist_speed = c.hist_speed;
        in.hist_yaw_rate = c.hist_yaw_rate;
        in.opp_present = c.opp_present;
        in.opp_x = c.opp_x;
        in.opp_y = c.opp_y;
        in.opp_yaw = c.opp_yaw;
        in.opp_vx = c.opp_vx;
        in.opp_vy = c.opp_vy;
        in.opp_yaw_rate = c.opp_yaw_rate;
        in.opp_position_sigma = c.opp_position_sigma;
        in.opp_heading_sigma = c.opp_heading_sigma;
        in.opp_yaw_rate_sigma = c.opp_yaw_rate_sigma;
        in.opp_half_diagonal = c.opp_half_diagonal;
        in.opp_age_s = c.opp_age_s;
        in.opp_stale = c.opp_stale;
        in.field_half_x = c.field_half_x;
        in.field_half_y = c.field_half_y;
        in.hazards = hazards;

        const Observation out = build(in);
        for (int i = 0; i < obs_v1::kSize; ++i) {
            EXPECT_NEAR(out[i], c.expected[i], kTolerance) << c.name << ", index " << i;
        }
    }
}

// --- Adapter -------------------------------------------------------------------------------

RobotDescription robot_at(FrameId frame_id, double x, double y, double yaw) {
    RobotDescription robot;
    robot.frame_id = frame_id;
    robot.pose = pose2d_to_pose(Pose2D{x, y, yaw});
    return robot;
}

FieldDescription field_of(double size_x, double size_y) {
    FieldDescription field;
    field.size.size = Size{size_x, size_y, 0.0};
    return field;
}

struct World {
    RobotDescription our = robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0, 0.0);
    std::optional<RobotDescription> opponent;
    FieldDescription field = field_of(2.4, 2.0);
    TargetSelection target;
    ActionHistory history{obs_v1::kHistory};

    Observation observe() {
        std::vector<ObservationHazardV1> storage;
        const ObservationContext context{our, opponent ? &*opponent : nullptr, field, target,
                                         history};
        return build(observation_inputs_v1(context, storage));
    }
};

TEST(ObservationV1AdapterTest, OurSpeedIsTheFieldVelocityAlongOurHeading) {
    World world;
    world.our = robot_at(FrameId::OUR_ROBOT_1, 0.1, -0.2, M_PI / 2.0);
    world.our.velocity = Velocity2D{0.3, 1.5, 4.0};  // mostly along +y, which is our heading
    world.our.measurement_age_s = 0.1;
    world.our.is_stale = true;
    world.target.mode = BehaviorMode::RUN_AWAY;

    const Observation out = world.observe();
    EXPECT_NEAR(out[kOur + 0], 1.5 / 2.5, 1e-12);
    EXPECT_NEAR(out[kOur + 1], 4.0 / 15.0, 1e-12);
    EXPECT_NEAR(out[kOur + 2], 0.1 / 0.5, 1e-12);
    EXPECT_EQ(out[kOur + 3], 1.0);
    EXPECT_EQ(out[kOur + 4], 0.0);  // ATTACK
    EXPECT_EQ(out[kOur + 5], 1.0);  // RUN_AWAY
}

TEST(ObservationV1AdapterTest, HistoryIsNewestFirst) {
    World world;
    world.history.push(HistoryEntry{0.1, 0.2, 1.0, 3.0});
    world.history.push(HistoryEntry{0.5, -0.6, 2.0, -6.0});

    const Observation out = world.observe();
    EXPECT_DOUBLE_EQ(out[kHistory + 0], 0.5);   // newest action, linear
    EXPECT_DOUBLE_EQ(out[kHistory + 1], -0.6);  // newest action, angular
    EXPECT_DOUBLE_EQ(out[kHistory + 2], 0.1);
    EXPECT_DOUBLE_EQ(out[kHistory + 3], 0.2);
    EXPECT_DOUBLE_EQ(out[kHistory + 16], 2.0 / 2.5);  // newest speed
    EXPECT_DOUBLE_EQ(out[kHistory + 17], -6.0 / 15);  // newest yaw rate
    EXPECT_DOUBLE_EQ(out[kHistory + 18], 1.0 / 2.5);
    EXPECT_DOUBLE_EQ(out[kHistory + 31], 0.0);  // never written
}

TEST(ObservationV1AdapterTest, AStaleTargetWithUnknownHeadingReadsTheCaps) {
    World world;
    RobotDescription opponent = robot_at(FrameId::THEIR_ROBOT_1, 1.0, 0.0, M_PI);
    opponent.is_stale = true;
    opponent.measurement_age_s = 0.9;
    opponent.size = Size{0.3, 0.4, 0.1};
    opponent.velocity = Velocity2D{-0.5, 0.25, 0.0};
    opponent.uncertainty.position_sigma_m = 0.03;
    // Heading and yaw-rate sigmas stay infinite: the heading was never measured.
    world.opponent = opponent;

    const Observation out = world.observe();
    EXPECT_NEAR(out[kOpponent + 0], 0.5, 1e-12);   // 1 m ahead, /2
    EXPECT_NEAR(out[kOpponent + 2], 0.5, 1e-12);   // distance /2
    EXPECT_NEAR(out[kOpponent + 4], 1.0, 1e-12);   // bearing cos: dead ahead
    EXPECT_NEAR(out[kOpponent + 6], -1.0, 1e-12);  // facing us
    EXPECT_NEAR(out[kOpponent + 7], -0.5 / 2.5, 1e-12);
    EXPECT_NEAR(out[kOpponent + 8], 0.25 / 2.5, 1e-12);
    EXPECT_NEAR(out[kOpponent + 10], 0.3, 1e-12);  // position sigma /0.1
    EXPECT_EQ(out[kOpponent + 11], 1.0);           // heading sigma: infinite reads the cap
    EXPECT_EQ(out[kOpponent + 12], 1.0);           // yaw-rate sigma: same
    EXPECT_NEAR(out[kOpponent + 13], 0.5 * std::hypot(0.3, 0.4) / 0.2, 1e-12);
    EXPECT_EQ(out[kOpponent + 14], 1.0);  // age 0.9 s caps at 1
    EXPECT_EQ(out[kOpponent + 15], 1.0);  // stale
}

TEST(ObservationV1AdapterTest, NoTargetReadsZerosAndUnknowns) {
    World world;
    const Observation out = world.observe();
    for (int i = 0; i < 10; ++i) EXPECT_EQ(out[kOpponent + i], 0.0) << i;
    EXPECT_EQ(out[kOpponent + 10], 1.0);
    EXPECT_EQ(out[kOpponent + 11], 1.0);
    EXPECT_EQ(out[kOpponent + 12], 1.0);
    EXPECT_EQ(out[kOpponent + 13], 0.0);
    EXPECT_EQ(out[kOpponent + 14], 1.0);
    EXPECT_EQ(out[kOpponent + 15], 1.0);
}

TEST(ObservationV1AdapterTest, NoHazardsLeavesEmptySlotsAndClearHoleRays) {
    World world;
    const Observation out = world.observe();
    for (int r = 0; r < obs_v1::kNumRays; ++r) {
        EXPECT_EQ(out[kRays + 2 * r + 1], 1.0) << "hole ray " << r;
    }
    for (int i = kHazards; i < obs_v1::kSize; ++i) EXPECT_EQ(out[i], 0.0) << i;
    // The field walls are still solid: 1.2 m ahead in a 2.4 m field, within the 1.5 m range.
    EXPECT_NEAR(out[kRays + 0], 1.2 / 1.5, 1e-12);
}

TEST(ObservationV1AdapterTest, OurRobotAgainstAWall) {
    World world;
    world.our = robot_at(FrameId::OUR_ROBOT_1, 1.2, 0.1, 0.0);  // on the +x wall, facing it
    const Observation out = world.observe();
    EXPECT_EQ(out[kRays + 0], 0.0);  // ray 0, dead ahead: touching
    // Ray 12 points back across the 2.4 m field, past the 1.5 m range.
    EXPECT_EQ(out[kRays + 2 * 12], 1.0);
    // Ray 6 points left (+y) to the wall 0.9 m away.
    EXPECT_NEAR(out[kRays + 2 * 6], 0.9 / 1.5, 1e-9);
}

TEST(ObservationV1AdapterTest, HazardsUseTheObjectRadiusAndTheirKind) {
    World world;
    FieldHazard house_bot;
    house_bot.center = Pose2D{0.5, 0.0, 0.0};
    house_bot.object_radius = 0.2;
    house_bot.inflated_radius = 0.5;  // never read
    house_bot.velocity = Velocity2D{0.25, 0.0, 0.0};
    house_bot.source = HazardSource::TRACKED;
    house_bot.kind = HazardKind::ROBOT;
    FieldHazard hole;
    hole.center = Pose2D{-0.8, 0.0, 0.0};
    hole.object_radius = 0.1;
    hole.kind = HazardKind::HOLE;
    world.field.hazards = {hole, house_bot};

    const Observation out = world.observe();
    // Slot 0 is the house bot: edge 0.3 m against the hole's 0.7 m.
    EXPECT_NEAR(out[kHazards + 0], 0.5 / 2.0, 1e-12);
    EXPECT_NEAR(out[kHazards + 2], 0.2 / 0.3, 1e-12);
    EXPECT_EQ(out[kHazards + 3], 0.0);
    EXPECT_EQ(out[kHazards + 4], 0.0);
    EXPECT_EQ(out[kHazards + 5], 1.0);  // tracked
    EXPECT_NEAR(out[kHazards + 6], 0.25 / 2.5, 1e-12);
    EXPECT_EQ(out[kHazards + 8], 1.0);
    // Slot 1 is the hole.
    EXPECT_NEAR(out[kHazards + 9], -0.8 / 2.0, 1e-12);
    EXPECT_EQ(out[kHazards + 12], 1.0);  // hole
    // The solid ray dead ahead stops on the house bot's raw edge, 0.3 m out.
    EXPECT_NEAR(out[kRays + 0], 0.3 / 1.5, 1e-9);
    // The hole ray straight back stops on the hole's edge, 0.7 m out.
    EXPECT_NEAR(out[kRays + 2 * 12 + 1], 0.7 / 1.5, 1e-9);
}

TEST(ObservationV1AdapterTest, BuilderWritesTheFloatTensor) {
    World world;
    world.our.velocity = Velocity2D{1.0, 0.0, 0.0};
    ObservationBuilderV1 builder;
    EXPECT_EQ(builder.output_specs(), (std::vector<TensorSpec>{TensorSpec{"obs.vector", {138}}}));
    TensorSet observation(builder.output_specs());
    const ObservationContext context{world.our, nullptr, world.field, world.target, world.history};
    builder.build(context, observation);
    const Observation expected = world.observe();
    for (int i = 0; i < obs_v1::kSize; ++i) {
        EXPECT_FLOAT_EQ(observation[0].data[i], static_cast<float>(expected[i])) << i;
    }
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
