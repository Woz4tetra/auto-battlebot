#include <gtest/gtest.h>
#include <cmath>
#include "navigation/pursuit_navigation.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot
{
    // Helper to create a robot at a given position with yaw
    RobotDescription make_robot(Label label, double x, double y, double yaw)
    {
        RobotDescription robot;
        robot.label = label;
        robot.pose.position.x = x;
        robot.pose.position.y = y;
        robot.pose.position.z = 0.0;
        robot.pose.rotation = euler_to_quaternion(0.0, 0.0, yaw);
        robot.velocity = Velocity2D{0.0, 0.0, 0.0};
        return robot;
    }

    RobotDescription make_robot_with_frame(
        Label label, FrameId frame_id, Group group, double x, double y, double yaw)
    {
        RobotDescription robot = make_robot(label, x, y, yaw);
        robot.frame_id = frame_id;
        robot.group = group;
        return robot;
    }

    // Helper to create a field description
    FieldDescription make_field(double width, double height)
    {
        FieldDescription field;
        field.size.size.x = width;
        field.size.size.y = height;
        return field;
    }

    TargetSelection make_target(double x, double y, Label label = Label::OPPONENT)
    {
        TargetSelection target;
        target.pose.x = x;
        target.pose.y = y;
        target.pose.yaw = 0.0;
        target.label = label;
        return target;
    }

    // Helper to create default config
    PursuitNavigationConfiguration make_default_config()
    {
        return PursuitNavigationConfiguration{};
    }

    class PursuitNavigationTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            config_ = make_default_config();
            nav_ = std::make_unique<PursuitNavigation>(config_);
            nav_->initialize();
        }

        PursuitNavigationConfiguration config_;
        std::unique_ptr<PursuitNavigation> nav_;
    };

    // ==================== Configuration Tests ====================

    TEST(PursuitNavigationConfigTest, DefaultValues)
    {
        PursuitNavigationConfiguration config;

        EXPECT_EQ(config.type, "PursuitNavigation");
        EXPECT_DOUBLE_EQ(config.slowdown_distance, 0.0);
        EXPECT_DOUBLE_EQ(config.stop_distance, 0.0);
        EXPECT_DOUBLE_EQ(config.angular_kp, 3.0);
        EXPECT_DOUBLE_EQ(config.angle_threshold, 0.5);
        EXPECT_DOUBLE_EQ(config.lookahead_time, 0.1);
        EXPECT_DOUBLE_EQ(config.boundary_margin, 0.1);
    }

    TEST(PursuitNavigationConfigTest, CustomValues)
    {
        PursuitNavigationConfiguration config;
        config.slowdown_distance = 1.0;
        config.stop_distance = 0.2;
        config.angular_kp = 3.0;
        config.angle_threshold = 0.5;
        config.lookahead_time = 0.2;
        config.boundary_margin = 0.15;

        PursuitNavigation nav(config);
        EXPECT_TRUE(nav.initialize());
    }

    // ==================== Initialization Tests ====================

    TEST_F(PursuitNavigationTest, InitializeReturnsTrue)
    {
        PursuitNavigationConfiguration config;
        PursuitNavigation nav(config);
        EXPECT_TRUE(nav.initialize());
    }

    // ==================== No Robots Tests ====================

    TEST_F(PursuitNavigationTest, NoRobotsReturnsZeroCommand)
    {
        RobotDescriptionsStamped robots;
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, 0.0, Label::EMPTY));

        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, NoOurRobotReturnsZeroCommand)
    {
        RobotDescriptionsStamped robots;
        // Only opponent, no "our" robot
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));

        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, UsesCachedOurRobotPoseWhenTemporarilyMissing)
    {
        FieldDescription field = make_field(4.0, 4.0);
        TargetSelection target = make_target(1.0, 0.0);

        RobotDescriptionsStamped first_tick_robots;
        first_tick_robots.descriptions.push_back(make_robot_with_frame(
            Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        first_tick_robots.descriptions.push_back(make_robot_with_frame(
            Label::OPPONENT, FrameId::THEIR_ROBOT_1, Group::THEIRS, 1.0, 0.0, 0.0));

        VelocityCommand first_cmd = nav_->update(first_tick_robots, field, target);
        EXPECT_GT(first_cmd.linear_x, 0.0);

        RobotDescriptionsStamped second_tick_robots;
        second_tick_robots.descriptions.push_back(make_robot_with_frame(
            Label::OPPONENT, FrameId::THEIR_ROBOT_1, Group::THEIRS, 1.0, 0.0, 0.0));

        VelocityCommand second_cmd = nav_->update(second_tick_robots, field, target);
        EXPECT_GT(second_cmd.linear_x, 0.0);
    }

    TEST_F(PursuitNavigationTest, MissingOurRobotWithoutCacheStillReturnsZeroCommand)
    {
        FieldDescription field = make_field(4.0, 4.0);
        TargetSelection target = make_target(1.0, 0.0);

        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(
            Label::OPPONENT, FrameId::THEIR_ROBOT_1, Group::THEIRS, 1.0, 0.0, 0.0));

        VelocityCommand cmd = nav_->update(robots, field, target);
        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, NoTargetReturnsZeroCommand)
    {
        RobotDescriptionsStamped robots;
        // Only our robot, no opponent
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, 0.0, Label::EMPTY));

        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    // ==================== Robot Detection Tests ====================

    TEST_F(PursuitNavigationTest, FindsOurRobotMrStabsMk1)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));

        // Should produce non-zero command since target exists
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, FindsOurRobotMrStabsMk2)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK2, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, FindsOurRobotMrsBuffMk1)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MRS_BUFF_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, FindsOurRobotMrsBuffMk2)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MRS_BUFF_MK2, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, TargetsOpponent)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, TargetsHouseBot)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::HOUSE_BOT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0, Label::HOUSE_BOT));
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, TargetsClosestOpponent)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Far opponent
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 2.0, 0.0, 0.0));
        // Close opponent
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.5, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(0.5, 0.0));

        // Should be moving forward toward the closer target
        EXPECT_GT(cmd.linear_x, 0.0);
    }

    // ==================== Movement Tests ====================

    TEST_F(PursuitNavigationTest, MovesForwardWhenFacingTarget)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Target directly ahead
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 0.0));

        EXPECT_GT(cmd.linear_x, 0.0);
        EXPECT_NEAR(cmd.angular_z, 0.0, 0.1); // Should be nearly zero rotation
    }

    TEST_F(PursuitNavigationTest, TurnsWhenTargetToLeft)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Target to the left (+Y direction)
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.0, 1.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, 1.0));

        // Should turn left (positive angular velocity in standard convention)
        EXPECT_GT(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, TurnsWhenTargetToRight)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Target to the right (-Y direction)
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.0, -1.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(0.0, -1.0));

        // Should turn right (negative angular velocity)
        EXPECT_LT(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, TurnsWhenTargetBehind)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Target behind (-X direction)
        robots.descriptions.push_back(make_robot(Label::OPPONENT, -1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(-1.0, 0.0));

        // Should turn (large angle error means turn in place)
        EXPECT_NEAR(cmd.linear_x, 0.0, 0.01);
        EXPECT_NE(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, StopsAtTarget)
    {
        RobotDescriptionsStamped robots;
        // Our robot very close to target (within stop_distance = 0.1)
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.05, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(0.05, 0.0));

        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, SlowsDownNearTarget)
    {
        // Create nav with known slowdown distance
        PursuitNavigationConfiguration config;
        config.slowdown_distance = 1.0;
        config.stop_distance = 0.1;
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        // Our robot facing target
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Target at half the slowdown distance
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.5, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav.update(robots, field, make_target(0.5, 0.0));

        // Should be slower than max velocity
        EXPECT_GT(cmd.linear_x, 0.0);
        EXPECT_LT(cmd.linear_x, 1.0);
    }

    // ==================== Velocity Limits Tests ====================

    TEST_F(PursuitNavigationTest, ForwardVelocityIsBoundedByControllerScaling)
    {
        PursuitNavigationConfiguration config;
        config.slowdown_distance = 0.1; // Small so we don't slow down
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 10.0, 0.0, 0.0));
        FieldDescription field = make_field(30.0, 30.0);

        VelocityCommand cmd = nav.update(robots, field, make_target(10.0, 0.0));

        EXPECT_LE(cmd.linear_x, 1.0);
    }

    TEST_F(PursuitNavigationTest, AngularVelocityUsesRealUnits)
    {
        PursuitNavigationConfiguration config;
        config.angular_kp = 2.0;
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));
        // Target behind to create large angle error
        robots.descriptions.push_back(make_robot(Label::OPPONENT, -1.0, 0.1, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav.update(robots, field, make_target(-1.0, 0.1));

        EXPECT_GT(std::abs(cmd.angular_z), 1.0);
    }

    // ==================== Target Prediction Tests ====================

    TEST_F(PursuitNavigationTest, NavigatesWithLookaheadTimeConfigured)
    {
        PursuitNavigationConfiguration config;
        config.lookahead_time = 1.0;
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X
        robots.descriptions.push_back(make_robot_with_frame(Label::MR_STABS_MK1, FrameId::OUR_ROBOT_1, Group::OURS, 0.0, 0.0, 0.0));

        // Target directly ahead
        RobotDescription target = make_robot(Label::OPPONENT, 1.0, 0.0, 0.0);
        target.velocity.vy = 1.0;
        robots.descriptions.push_back(target);

        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav.update(robots, field, make_target(1.0, 0.0));

        // Target is at (1, 0), robot faces +X: drives forward
        EXPECT_GT(cmd.linear_x, 0.0);
    }

    // ==================== Boundary Tests ====================

    TEST_F(PursuitNavigationTest, ClampsTargetToFieldBoundary)
    {
        PursuitNavigationConfiguration config;
        config.boundary_margin = 0.1;
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target outside field boundary
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 10.0, 0.0, 0.0));

        // Small field
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav.update(robots, field, make_target(10.0, 0.0));

        // Should still produce valid command (toward clamped position)
        EXPECT_TRUE(cmd.linear_x >= 0.0 || cmd.angular_z != 0.0);
    }

    // ==================== Linear Y Velocity Tests ====================

    TEST_F(PursuitNavigationTest, LinearYIsAlwaysZero)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 1.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field, make_target(1.0, 1.0));

        // This is a differential drive robot - no lateral movement
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
    }

} // namespace auto_battlebot
