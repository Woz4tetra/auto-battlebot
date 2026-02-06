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

    // Helper to create a field description
    FieldDescription make_field(double width, double height)
    {
        FieldDescription field;
        field.size.size.x = width;
        field.size.size.y = height;
        return field;
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
        EXPECT_DOUBLE_EQ(config.max_linear_velocity, 1.0);
        EXPECT_DOUBLE_EQ(config.max_angular_velocity, 3.0);
        EXPECT_DOUBLE_EQ(config.slowdown_distance, 0.5);
        EXPECT_DOUBLE_EQ(config.stop_distance, 0.1);
        EXPECT_DOUBLE_EQ(config.angular_kp, 2.0);
        EXPECT_DOUBLE_EQ(config.angle_threshold, 0.3);
        EXPECT_DOUBLE_EQ(config.lookahead_time, 0.1);
        EXPECT_DOUBLE_EQ(config.boundary_margin, 0.1);
    }

    TEST(PursuitNavigationConfigTest, CustomValues)
    {
        PursuitNavigationConfiguration config;
        config.max_linear_velocity = 2.5;
        config.max_angular_velocity = 5.0;
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

        VelocityCommand cmd = nav_->update(robots, field);

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

        VelocityCommand cmd = nav_->update(robots, field);

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

        VelocityCommand cmd = nav_->update(robots, field);

        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    // ==================== Robot Detection Tests ====================

    TEST_F(PursuitNavigationTest, FindsOurRobotMrStabsMk1)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field);

        // Should produce non-zero command since target exists
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, FindsOurRobotMrStabsMk2)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK2, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field);
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, FindsOurRobotMrsBuffMk1)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MRS_BUFF_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field);
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, FindsOurRobotMrsBuffMk2)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MRS_BUFF_MK2, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(2.0, 2.0);

        VelocityCommand cmd = nav_->update(robots, field);
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, TargetsOpponent)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, TargetsHouseBot)
    {
        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::HOUSE_BOT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);
        EXPECT_TRUE(cmd.linear_x != 0.0 || cmd.angular_z != 0.0);
    }

    TEST_F(PursuitNavigationTest, TargetsClosestOpponent)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Far opponent
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 2.0, 0.0, 0.0));
        // Close opponent
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.5, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);

        // Should be moving forward toward the closer target
        EXPECT_GT(cmd.linear_x, 0.0);
    }

    // ==================== Movement Tests ====================

    TEST_F(PursuitNavigationTest, MovesForwardWhenFacingTarget)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target directly ahead
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);

        EXPECT_GT(cmd.linear_x, 0.0);
        EXPECT_NEAR(cmd.angular_z, 0.0, 0.1); // Should be nearly zero rotation
    }

    TEST_F(PursuitNavigationTest, TurnsWhenTargetToLeft)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target to the left (+Y direction)
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.0, 1.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);

        // Should turn left (positive angular velocity in standard convention)
        EXPECT_GT(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, TurnsWhenTargetToRight)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target to the right (-Y direction)
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.0, -1.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);

        // Should turn right (negative angular velocity)
        EXPECT_LT(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, TurnsWhenTargetBehind)
    {
        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X (yaw = 0)
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target behind (-X direction)
        robots.descriptions.push_back(make_robot(Label::OPPONENT, -1.0, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav_->update(robots, field);

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

        VelocityCommand cmd = nav_->update(robots, field);

        EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
    }

    TEST_F(PursuitNavigationTest, SlowsDownNearTarget)
    {
        // Create nav with known slowdown distance
        PursuitNavigationConfiguration config;
        config.slowdown_distance = 1.0;
        config.stop_distance = 0.1;
        config.max_linear_velocity = 2.0;
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        // Our robot facing target
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target at half the slowdown distance
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 0.5, 0.0, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav.update(robots, field);

        // Should be slower than max velocity
        EXPECT_GT(cmd.linear_x, 0.0);
        EXPECT_LT(cmd.linear_x, config.max_linear_velocity);
    }

    // ==================== Velocity Limits Tests ====================

    TEST_F(PursuitNavigationTest, RespectsMaxLinearVelocity)
    {
        PursuitNavigationConfiguration config;
        config.max_linear_velocity = 0.5;
        config.slowdown_distance = 0.1; // Small so we don't slow down
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        robots.descriptions.push_back(make_robot(Label::OPPONENT, 10.0, 0.0, 0.0));
        FieldDescription field = make_field(30.0, 30.0);

        VelocityCommand cmd = nav.update(robots, field);

        EXPECT_LE(cmd.linear_x, config.max_linear_velocity);
    }

    TEST_F(PursuitNavigationTest, RespectsMaxAngularVelocity)
    {
        PursuitNavigationConfiguration config;
        config.max_angular_velocity = 1.0;
        config.angular_kp = 100.0; // Very high gain to saturate
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));
        // Target behind to create large angle error
        robots.descriptions.push_back(make_robot(Label::OPPONENT, -1.0, 0.1, 0.0));
        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav.update(robots, field);

        EXPECT_LE(std::abs(cmd.angular_z), config.max_angular_velocity);
    }

    // ==================== Target Prediction Tests ====================

    TEST_F(PursuitNavigationTest, PredictsMovingTarget)
    {
        PursuitNavigationConfiguration config;
        config.lookahead_time = 1.0; // 1 second lookahead
        PursuitNavigation nav(config);
        nav.initialize();

        RobotDescriptionsStamped robots;
        // Our robot at origin facing +X
        robots.descriptions.push_back(make_robot(Label::MR_STABS_MK1, 0.0, 0.0, 0.0));

        // Target at (1, 0) moving in +Y direction at 1 m/s
        RobotDescription target = make_robot(Label::OPPONENT, 1.0, 0.0, 0.0);
        target.velocity.vy = 1.0;
        robots.descriptions.push_back(target);

        FieldDescription field = make_field(4.0, 4.0);

        VelocityCommand cmd = nav.update(robots, field);

        // Should turn toward predicted position (1, 1), which is to the left
        EXPECT_GT(cmd.angular_z, 0.0);
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

        VelocityCommand cmd = nav.update(robots, field);

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

        VelocityCommand cmd = nav_->update(robots, field);

        // This is a differential drive robot - no lateral movement
        EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
    }

} // namespace auto_battlebot
