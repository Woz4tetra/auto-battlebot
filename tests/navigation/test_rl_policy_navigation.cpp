// RlPolicyNavigation: the startup checks that refuse a bad bundle or a HOLD opponent filter, the
// no_robot path, the policy schedule, and which opponent it hands the policy.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "navigation/config.hpp"
#include "navigation/rl_policy_navigation.hpp"
#include "rl_policy/bundle_writer.hpp"
#include "robot_filter/config.hpp"
#include "time/manual_clock.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {

using rl_policy::PolicyBundleError;

// Observation v1 indices the test networks read.
constexpr int kOurSpeed = 0;
constexpr int kOurYawRate = 1;
constexpr int kOpponentX = 38;

RlPolicyNavigationConfiguration config_for(
    const std::filesystem::path &bundle, std::optional<OpponentMode> mode = OpponentMode::KALMAN) {
    RlPolicyNavigationConfiguration config;
    config.policy.bundle = bundle.string();
    config.opponent_mode = mode;
    config.opponent_mode_source = "the test";
    return config;
}

std::unique_ptr<RlPolicyNavigation> make_nav(const RlPolicyNavigationConfiguration &config) {
    return std::make_unique<RlPolicyNavigation>(config, std::make_shared<ManualClock>());
}

RobotDescription robot_at(FrameId frame_id, double x, double y, double yaw = 0.0) {
    RobotDescription robot;
    robot.frame_id = frame_id;
    robot.group = frame_id == FrameId::OUR_ROBOT_1 ? Group::OURS : Group::THEIRS;
    robot.pose = pose2d_to_pose(Pose2D{x, y, yaw});
    return robot;
}

RobotDescriptionsStamped robots_of(std::vector<RobotDescription> descriptions) {
    RobotDescriptionsStamped robots;
    robots.descriptions = std::move(descriptions);
    return robots;
}

FieldDescription field() {
    FieldDescription field;
    field.size.size = Size{2.4, 2.4, 0.0};
    return field;
}

/** Message of the exception `initialize()` throws, or "" when it starts. */
template <typename Error>
std::string start_error(const RlPolicyNavigationConfiguration &config) {
    auto nav = make_nav(config);
    try {
        nav->initialize();
    } catch (const Error &e) {
        return e.what();
    }
    return "";
}

TEST(RlPolicyNavigationTest, NoRobotCommandsZeroAndLogsIt) {
    auto nav = make_nav(config_for(rl_test::constant_bundle()));
    ASSERT_TRUE(nav->initialize());
    auto logger = DiagnosticsLogger::get_logger("rl_policy_nav");
    logger->clear();

    const VelocityCommand command = nav->update(robots_of({}), field(), TargetSelection{});
    EXPECT_EQ(command.linear_x, 0.0);
    EXPECT_EQ(command.linear_y, 0.0);
    EXPECT_EQ(command.angular_z, 0.0);
    bool logged = false;
    for (const auto &snapshot : logger->get_snapshots()) {
        logged = logged || (snapshot.subsection == "no_robot" && !snapshot.held);
    }
    EXPECT_TRUE(logged);
    EXPECT_FALSE(nav->get_last_visualization().path.has_value());
}

TEST(RlPolicyNavigationTest, AnUnmeasuredRobotCommandsZero) {
    auto nav = make_nav(config_for(rl_test::constant_bundle()));
    ASSERT_TRUE(nav->initialize());
    auto logger = DiagnosticsLogger::get_logger("rl_policy_nav");
    logger->clear();

    RobotDescription ours = robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0);
    ours.velocity.omega = 15.0;
    ours.measurement_age_s = RlPolicyNavigationConfiguration{}.max_our_unmeasured_s + 0.05;
    const VelocityCommand command = nav->update(robots_of({ours}), field(), TargetSelection{});
    EXPECT_EQ(command.linear_x, 0.0);
    EXPECT_EQ(command.angular_z, 0.0);
    bool logged = false;
    for (const auto &snapshot : logger->get_snapshots()) {
        logged = logged || (snapshot.subsection == "our_robot_unmeasured" && !snapshot.held);
    }
    EXPECT_TRUE(logged);

    // Measured again: the policy steps on that same tick rather than waiting out its period.
    ours.measurement_age_s = 0.0;
    const VelocityCommand resumed = nav->update(robots_of({ours}), field(), TargetSelection{});
    EXPECT_DOUBLE_EQ(resumed.linear_x, 0.3);
    EXPECT_DOUBLE_EQ(resumed.angular_z, -0.1);
}

TEST(RlPolicyNavigationTest, AFrozenSnapshotAgesOnTheControlClock) {
    // The cache hands back a snapshot whose measurement_age_s stopped growing when it was cached.
    auto clock = std::make_shared<ManualClock>();
    RlPolicyNavigation nav(config_for(rl_test::constant_bundle()), clock);
    ASSERT_TRUE(nav.initialize());

    RobotDescriptionsStamped robots = robots_of({robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0)});
    robots.header.stamp = 10.0;
    robots.descriptions[0].measurement_age_s = 0.05;
    clock->set(10.05);
    EXPECT_DOUBLE_EQ(nav.update(robots, field(), TargetSelection{}).linear_x, 0.3);

    clock->set(10.0 + RlPolicyNavigationConfiguration{}.max_our_unmeasured_s);
    EXPECT_EQ(nav.update(robots, field(), TargetSelection{}).linear_x, 0.0);
}

TEST(RlPolicyNavigationTest, RefusesHoldOpponentMode) {
    const std::string error = start_error<ConfigValidationError>(
        config_for(rl_test::constant_bundle(), OpponentMode::HOLD));
    EXPECT_NE(error.find("opponent_mode = 'KALMAN'"), std::string::npos) << error;
    EXPECT_NE(error.find("Found opponent_mode HOLD"), std::string::npos) << error;
}

TEST(RlPolicyNavigationTest, RefusesAFilterWithNoOpponentMode) {
    const std::string error =
        start_error<ConfigValidationError>(config_for(rl_test::constant_bundle(), std::nullopt));
    EXPECT_NE(error.find("Found opponent_mode not set"), std::string::npos) << error;
}

TEST(RlPolicyNavigationTest, AFailedSelftestRefusesToStart) {
    const auto bundle = rl_test::write_linear_bundle("nav_selftest_fail", 1, 138, 0.5, 0.01);
    const std::string error = start_error<PolicyBundleError>(config_for(bundle));
    EXPECT_NE(error.find("self-test failed"), std::string::npos) << error;
}

TEST(RlPolicyNavigationTest, RefusesASpecVersionWithNoBuilder) {
    // tiny_mlp is obs spec 0: a unit-test bundle with no builder, never for navigation.
    const std::string error =
        start_error<PolicyBundleError>(config_for(rl_test::tiny_mlp_bundle()));
    EXPECT_NE(error.find("obs_spec_version 0 has no observation builder"), std::string::npos)
        << error;
}

TEST(RlPolicyNavigationTest, RefusesANetworkInputTheBuilderDoesNotMake) {
    const auto renamed =
        rl_test::write_linear_bundle("nav_input_name", 1, 138, 0.5, 0.0, "obs.other");
    const std::string name_error = start_error<PolicyBundleError>(config_for(renamed));
    EXPECT_NE(name_error.find("the network reads obs.other [138] but the obs_spec_version 1 "
                              "builder makes obs.vector [138]"),
              std::string::npos)
        << name_error;

    const auto resized = rl_test::write_linear_bundle("nav_input_shape", 1, 137);
    const std::string shape_error = start_error<PolicyBundleError>(config_for(resized));
    EXPECT_NE(shape_error.find("the network reads obs.vector [137]"), std::string::npos)
        << shape_error;
}

TEST(RlPolicyNavigationTest, AMissingBundleRefusesToStart) {
    const std::string error =
        start_error<PolicyBundleError>(config_for("/nonexistent/rl_nav_bundle"));
    EXPECT_NE(error.find("no manifest.toml"), std::string::npos) << error;
}

TEST(RlPolicyNavigationTest, ConstantPolicyDrivesItsActionUnedited) {
    auto nav = make_nav(config_for(rl_test::constant_bundle()));
    nav->initialize();
    TargetSelection target;
    target.pose = Pose2D{1.0, 0.5, 0.0};
    const VelocityCommand command =
        nav->update(robots_of({robot_at(FrameId::OUR_ROBOT_1, -0.2, 0.1)}), field(), target);
    EXPECT_DOUBLE_EQ(command.linear_x, 0.3);
    EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
    EXPECT_DOUBLE_EQ(command.angular_z, -0.1);

    const auto &viz = nav->get_last_visualization();
    ASSERT_TRUE(viz.path.has_value());
    EXPECT_DOUBLE_EQ(viz.path->our_x, -0.2);
    EXPECT_DOUBLE_EQ(viz.path->our_y, 0.1);
    EXPECT_DOUBLE_EQ(viz.path->target_x, 1.0);
    EXPECT_DOUBLE_EQ(viz.path->target_y, 0.5);
    EXPECT_DOUBLE_EQ(viz.command.linear_x, 0.3);
}

TEST(RlPolicyNavigationTest, TheHazardBarrierClipsDrivingTowardAHole) {
    RlPolicyNavigationConfiguration config = config_for(rl_test::constant_bundle());
    config.hazard_barrier_enable = true;
    JigPlantParams plant;
    plant.k_fwd = 4.9;
    plant.k_rev = 4.4;
    plant.tau_lin_d = 0.2;
    plant.delay_s = 0.05;
    config.apply_plant(PlantConfiguration{plant, {}});
    auto nav = make_nav(config);
    nav->initialize();

    FieldDescription with_hole = field();
    FieldHazard hole;
    hole.center = Pose2D{0.25, 0.0, 0.0};
    hole.hard_radius = 0.2;
    with_hole.hazards.push_back(hole);

    // 5 cm from the hard rim, facing it: 0.3 of 4.9 m/s cannot stop in time.
    const VelocityCommand toward =
        nav->update(robots_of({robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0)}), with_hole, {});
    EXPECT_LT(toward.linear_x, 0.3);
    EXPECT_DOUBLE_EQ(toward.angular_z, -0.1);

    // Facing away, the policy's forward command leaves the hole behind and passes unedited.
    const VelocityCommand away =
        nav->update(robots_of({robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0, M_PI)}), with_hole, {});
    EXPECT_DOUBLE_EQ(away.linear_x, 0.3);
}

TEST(RlPolicyNavigationConfigTest, TheHazardBarrierNeedsAPlant) {
    RlPolicyNavigationConfiguration config = config_for(rl_test::constant_bundle());
    config.hazard_barrier_enable = true;
    EXPECT_THROW(config.apply_plant(PlantConfiguration{}), ConfigValidationError);
}

TEST(RlPolicyNavigationTest, StepsThePolicyEveryPeriodAndHoldsTheActionBetween) {
    // linear = tanh(0.5 * our_speed / 2.5), angular = tanh(0.5 * our_yaw_rate / 15).
    const auto bundle = rl_test::write_linear_bundle("nav_schedule", 1, 138, 0.5, 0.0, "obs.vector",
                                                     kOurSpeed, kOurYawRate);
    auto nav = make_nav(config_for(bundle));
    nav->initialize();

    const auto tick = [&](double speed, double yaw_rate) {
        RobotDescription our = robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0);
        our.velocity = Velocity2D{speed, 0.0, yaw_rate};
        return nav->update(robots_of({our}), field(), TargetSelection{});
    };

    const VelocityCommand first = tick(1.0, 3.0);
    EXPECT_NEAR(first.linear_x, std::tanh(0.5 * 1.0 / 2.5), 1e-6);
    EXPECT_NEAR(first.angular_z, std::tanh(0.5 * 3.0 / 15.0), 1e-6);
    // The bundle's policy_period_ticks is 5: four more ticks repeat the stored action.
    for (int i = 0; i < 4; ++i) {
        const VelocityCommand held = tick(2.0, -3.0);
        EXPECT_DOUBLE_EQ(held.linear_x, first.linear_x) << "tick " << i + 1;
        EXPECT_DOUBLE_EQ(held.angular_z, first.angular_z) << "tick " << i + 1;
    }
    const VelocityCommand next = tick(2.0, -3.0);
    EXPECT_NEAR(next.linear_x, std::tanh(0.5 * 2.0 / 2.5), 1e-6);
    EXPECT_NEAR(next.angular_z, std::tanh(0.5 * -3.0 / 15.0), 1e-6);
}

TEST(RlPolicyNavigationTest, TheTargetOpponentFollowsTheBehaviourMode) {
    // linear = tanh(0.5 * opponent x in our body frame / 2).
    const auto bundle = rl_test::write_linear_bundle("nav_target", 1, 138, 0.5, 0.0, "obs.vector",
                                                     kOpponentX, kOurYawRate);
    const auto robots = robots_of({robot_at(FrameId::OUR_ROBOT_1, 0.0, 0.0),
                                   robot_at(FrameId::THEIR_ROBOT_1, 1.0, 0.0),
                                   robot_at(FrameId::THEIR_ROBOT_2, -0.5, 0.0)});
    TargetSelection target;
    target.pose = Pose2D{1.0, 0.1, 0.0};  // the selector picked THEIR_ROBOT_1

    // ATTACK: the opponent nearest the selected target.
    auto attack = make_nav(config_for(bundle));
    attack->initialize();
    target.mode = BehaviorMode::ATTACK;
    EXPECT_NEAR(attack->update(robots, field(), target).linear_x, std::tanh(0.5 * 1.0 / 2.0), 1e-6);

    // RUN_AWAY: the target pose is a safe point, so the opponent is the one nearest us.
    auto run_away = make_nav(config_for(bundle));
    run_away->initialize();
    target.mode = BehaviorMode::RUN_AWAY;
    EXPECT_NEAR(run_away->update(robots, field(), target).linear_x, std::tanh(0.5 * -0.5 / 2.0),
                1e-6);
}

TEST(RlPolicyNavigationConfigTest, ParsesThePolicyTable) {
    const toml::table table = toml::parse(R"(
type = "RlPolicyNavigation"
[policy]
bundle = "data/models/nav_policies/rl_nav_test"
)");
    ConfigParser parser(table, "navigation");
    auto config = parse_navigation_config(parser);
    auto *rl = dynamic_cast<RlPolicyNavigationConfiguration *>(config.get());
    ASSERT_NE(rl, nullptr);
    EXPECT_EQ(rl->policy.bundle, "data/models/nav_policies/rl_nav_test");

    const toml::table missing = toml::parse("type = \"RlPolicyNavigation\"\n");
    ConfigParser missing_parser(missing, "navigation");
    EXPECT_THROW(parse_navigation_config(missing_parser), ConfigValidationError);
}

TEST(RlPolicyNavigationConfigTest, ReadsTheOpponentModeFromTheRobotFilter) {
    RobotFrontBackFilterConfiguration front_back;
    auto kalman = std::make_shared<KalmanMotionEstimatorConfiguration>();
    kalman->opponent_mode = OpponentMode::KALMAN;
    front_back.motion_estimator = kalman;
    RlPolicyNavigationConfiguration config;
    config.apply_robot_filter(front_back);
    EXPECT_EQ(config.opponent_mode, OpponentMode::KALMAN);

    kalman->opponent_mode = OpponentMode::HOLD;
    config.apply_robot_filter(front_back);
    EXPECT_EQ(config.opponent_mode, OpponentMode::HOLD);

    front_back.motion_estimator = std::make_shared<DeadReckoningMotionEstimatorConfiguration>();
    config.apply_robot_filter(front_back);
    EXPECT_FALSE(config.opponent_mode.has_value());
    EXPECT_NE(config.opponent_mode_source.find("DeadReckoningMotionEstimator"), std::string::npos);

    config.apply_robot_filter(GroundTruthRobotFilterConfiguration{});
    EXPECT_FALSE(config.opponent_mode.has_value());
    EXPECT_NE(config.opponent_mode_source.find("GroundTruthRobotFilter"), std::string::npos);
}

TEST(RlPolicyNavigationConfigTest, MakeNavigationRunsTheStartupChecks) {
    // A bad bundle has to stop the app on the main thread, before the control loop starts.
    EXPECT_THROW(make_navigation(config_for(rl_test::constant_bundle(), OpponentMode::HOLD),
                                 std::make_shared<ManualClock>()),
                 ConfigValidationError);
    EXPECT_NE(
        make_navigation(config_for(rl_test::constant_bundle()), std::make_shared<ManualClock>()),
        nullptr);
}

}  // namespace
}  // namespace auto_battlebot
