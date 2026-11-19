
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

#include "rl_policy/allocation_counter.hpp"
#include "rl_policy/observations/observation_v1.hpp"
#include "rl_policy/rl_policy_golden_data.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot::rl_policy {
namespace {

constexpr double kTolerance = 1e-5;

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
        in.our_forward_speed_mps = c.our_speed;
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
    opponent.uncertainty.position_sigma_major_axis_m = 0.03;
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
    house_bot.inflated_radius_m = 0.5;  // never read
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
        EXPECT_FLOAT_EQ(observation[0].values()[i], static_cast<float>(expected[i])) << i;
    }
}

TEST(ObservationV1AdapterTest, BuildDoesNotAllocateAfterWarmup) {
    World world;
    world.opponent = robot_at(FrameId::THEIR_ROBOT_1, 0.5, 0.2, 1.0);
    ObservationBuilderV1 builder;
    TensorSet observation(builder.output_specs());
    const ObservationContext context{world.our, &*world.opponent, world.field, world.target,
                                     world.history};
    builder.build(context, observation);

    const long allocations = rl_test::allocations_during([&] {
        for (int i = 0; i < 100; ++i) builder.build(context, observation);
    });
    EXPECT_EQ(allocations, 0);
}

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

ObservationInputsV1 plain_inputs() {
    ObservationInputsV1 in;
    in.field_half_x = 0.9;
    in.field_half_y = 0.9;
    return in;
}

TEST(ObservationV1SpecTest, CapMapsNanAndPositiveInfinityToOneAndNegativeInfinityToZero) {
    ObservationInputsV1 in = plain_inputs();
    in.our_age_s = kNan;
    EXPECT_EQ(build(in)[2], 1.0);
    in.our_age_s = kInfinity;
    EXPECT_EQ(build(in)[2], 1.0);
    in.our_age_s = -kInfinity;
    EXPECT_EQ(build(in)[2], 0.0);
}

TEST(ObservationV1SpecTest, WallRayFromOutsideOrOnTheWallReadsZero) {
    ObservationInputsV1 in = plain_inputs();
    in.our_x = 2.0;
    EXPECT_EQ(build(in)[kRays], 0.0);
    in.our_x = 0.9;
    EXPECT_EQ(build(in)[kRays], 0.0);
}

TEST(ObservationV1SpecTest, HazardKindOneHotOrderMatchesTraining) {
    constexpr int kTrainedHole = 0;
    constexpr int kTrainedWallBlock = 1;
    constexpr int kTrainedRobot = 2;
    for (int slot : {kTrainedHole, kTrainedWallBlock, kTrainedRobot}) {
        ObservationInputsV1 in = plain_inputs();
        const std::vector<ObservationHazardV1> hazards = {
            ObservationHazardV1{0.5, 0.0, 0.2, static_cast<HazardKind>(slot), 0.0, 0.0, true}};
        in.hazards = hazards;
        const Observation out = build(in);
        for (int i = 0; i < 3; ++i) {
            EXPECT_EQ(out[kHazards + 3 + i], i == slot ? 1.0 : 0.0)
                << "enum value " << slot << " one-hot index " << i;
        }
    }
}

TEST(ObservationV1SpecTest, HoleRayFromInsideADiscReadsZero) {
    ObservationInputsV1 in = plain_inputs();
    const std::vector<ObservationHazardV1> hazards = {
        ObservationHazardV1{0.1, 0.0, 0.3, HazardKind::HOLE, 0.0, 0.0, true}};
    in.hazards = hazards;
    const Observation out = build(in);
    EXPECT_EQ(out[kRays + 1], 0.0);
    EXPECT_EQ(out[kRays + 2 * 12 + 1], 0.0);
}

TEST(ObservationV1SpecTest, AbsentOpponentReadsZerosExceptSigmasAgeAndStale) {
    ObservationInputsV1 in = plain_inputs();
    in.opp_present = false;
    in.opp_x = 0.7;
    in.opp_y = -0.4;
    in.opp_yaw = 1.0;
    in.opp_vx = 1.0;
    in.opp_yaw_rate = 3.0;
    in.opp_half_diagonal = 0.2;
    const Observation out = build(in);
    for (int i = 0; i < 10; ++i) EXPECT_EQ(out[kOpponent + i], 0.0) << i;
    for (int i = 10; i < 13; ++i) EXPECT_EQ(out[kOpponent + i], 1.0) << i;
    EXPECT_EQ(out[kOpponent + 13], 0.0);
    EXPECT_EQ(out[kOpponent + 14], 1.0);
    EXPECT_EQ(out[kOpponent + 15], 1.0);
}

TEST(ObservationV1SpecTest, HazardSlotsSortAscendingWithNanLastAndTiesInInputOrder) {
    ObservationInputsV1 in = plain_inputs();
    const std::vector<ObservationHazardV1> hazards = {
        ObservationHazardV1{1.0, 0.0, 0.5, HazardKind::ROBOT, 0.0, 0.0, true},
        ObservationHazardV1{kNan, 0.0, 0.0625, HazardKind::ROBOT, 0.0, 0.0, true},
        ObservationHazardV1{0.75, 0.0, 0.25, HazardKind::ROBOT, 0.0, 0.0, true},
        ObservationHazardV1{0.375, 0.0, 0.125, HazardKind::ROBOT, 0.0, 0.0, true}};
    in.hazards = hazards;
    const Observation out = build(in);
    const double expected_radii[4] = {0.125, 0.5, 0.25, 0.0625};
    for (int k = 0; k < 4; ++k) {
        EXPECT_DOUBLE_EQ(out[kHazards + 9 * k + 2], expected_radii[k] / 0.3) << k;
    }
}

TEST(TensorSetTest, FindDoesNotAllocate) {
    TensorSet set({TensorSpec{"obs.vector", {138}}, TensorSpec{"action", {2}}});
    const std::string_view name = "action";
    const long allocations = rl_test::allocations_during([&] {
        for (int i = 0; i < 100; ++i) EXPECT_NE(set.find(name), nullptr);
    });
    EXPECT_EQ(allocations, 0);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
