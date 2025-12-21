#include <gtest/gtest.h>
#include "ros/ros_message_adapters/ros_marker.hpp"
#include <visualization_msgs/Marker.hxx>

namespace auto_battlebot
{
    class RosMarkerAdapterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            test_header.stamp = 123.456;
            test_header.frame_id = FrameId::CAMERA;

            // Setup field description
            field_desc.header = test_header;
            field_desc.size.header = test_header;
            field_desc.size.size.x = 2.0;
            field_desc.size.size.y = 3.0;
            field_desc.size.size.z = 0.1;

            // Create transform for field
            field_desc.tf_fieldcenter_from_camera.tf = Eigen::MatrixXd::Identity(4, 4);
            field_desc.tf_fieldcenter_from_camera.tf(0, 3) = 1.0;
            field_desc.tf_fieldcenter_from_camera.tf(1, 3) = 2.0;
            field_desc.tf_fieldcenter_from_camera.tf(2, 3) = 3.0;

            // Setup keypoints
            keypoints.header = test_header;
            Keypoint kp1;
            kp1.label = "robot1";
            kp1.keypoint_label = "center";
            kp1.x = 10.0f;
            kp1.y = 20.0f;
            keypoints.keypoints.push_back(kp1);

            Keypoint kp2;
            kp2.label = "robot2";
            kp2.keypoint_label = "corner";
            kp2.x = 30.0f;
            kp2.y = 40.0f;
            keypoints.keypoints.push_back(kp2);

            // Setup robots
            robots.header = test_header;
            RobotDescription robot1;
            robot1.label = Label::MR_STABS_MK1;
            robot1.group = Group::OURS;
            robot1.pose.position.x = 1.0;
            robot1.pose.position.y = 2.0;
            robot1.pose.position.z = 0.5;
            robot1.pose.rotation.w = 1.0;
            robot1.size.x = 0.3;
            robot1.size.y = 0.3;
            robot1.size.z = 0.2;
            robots.descriptions.push_back(robot1);

            RobotDescription robot2;
            robot2.label = Label::MR_STABS_MK1;
            robot2.group = Group::THEIRS;
            robot2.pose.position.x = -1.0;
            robot2.pose.position.y = -2.0;
            robot2.pose.position.z = 0.5;
            robot2.pose.rotation.w = 1.0;
            robot2.size.x = 0.3;
            robot2.size.y = 0.3;
            robot2.size.z = 0.2;
            robots.descriptions.push_back(robot2);
        }

        Header test_header;
        FieldDescription field_desc;
        KeypointsStamped keypoints;
        RobotDescriptionsStamped robots;
    };

    // Test field marker conversion
    TEST_F(RosMarkerAdapterTest, FieldMarkerConversion)
    {
        auto marker = ros_adapters::to_ros_field_marker(field_desc);

        EXPECT_EQ(marker.ns, "field");
        EXPECT_EQ(marker.id, 0);
        EXPECT_EQ(marker.type, visualization_msgs::Marker::CUBE);
        EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);

        // Check pose
        EXPECT_DOUBLE_EQ(marker.pose.position.x, 1.0);
        EXPECT_DOUBLE_EQ(marker.pose.position.y, 2.0);
        EXPECT_DOUBLE_EQ(marker.pose.position.z, 3.0);

        // Check scale
        EXPECT_DOUBLE_EQ(marker.scale.x, 2.0);
        EXPECT_DOUBLE_EQ(marker.scale.y, 3.0);
        EXPECT_DOUBLE_EQ(marker.scale.z, 0.01); // Thin for field

        // Check color (green field)
        EXPECT_FLOAT_EQ(marker.color.r, 0.0f);
        EXPECT_FLOAT_EQ(marker.color.g, 1.0f);
        EXPECT_FLOAT_EQ(marker.color.b, 0.0f);
        EXPECT_FLOAT_EQ(marker.color.a, 0.5f);
    }

    // Test keypoint markers conversion
    TEST_F(RosMarkerAdapterTest, KeypointMarkersConversion)
    {
        auto markers = ros_adapters::to_ros_keypoint_markers(keypoints);

        EXPECT_EQ(markers.size(), 2);

        // Check first marker
        EXPECT_EQ(markers[0].ns, "keypoints");
        EXPECT_EQ(markers[0].id, 0);
        EXPECT_EQ(markers[0].type, visualization_msgs::Marker::SPHERE);
        EXPECT_DOUBLE_EQ(markers[0].pose.position.x, 10.0);
        EXPECT_DOUBLE_EQ(markers[0].pose.position.y, 20.0);
        EXPECT_DOUBLE_EQ(markers[0].pose.position.z, 0.0);

        // Check second marker
        EXPECT_EQ(markers[1].ns, "keypoints");
        EXPECT_EQ(markers[1].id, 1);
        EXPECT_DOUBLE_EQ(markers[1].pose.position.x, 30.0);
        EXPECT_DOUBLE_EQ(markers[1].pose.position.y, 40.0);
    }

    // Test robot markers conversion
    TEST_F(RosMarkerAdapterTest, RobotMarkersConversion)
    {
        auto markers = ros_adapters::to_ros_robot_markers(robots);

        EXPECT_EQ(markers.size(), 2);

        // Check first robot (ours - blue)
        EXPECT_EQ(markers[0].ns, "robots");
        EXPECT_EQ(markers[0].id, 0);
        EXPECT_EQ(markers[0].type, visualization_msgs::Marker::CUBE);
        EXPECT_DOUBLE_EQ(markers[0].pose.position.x, 1.0);
        EXPECT_DOUBLE_EQ(markers[0].pose.position.y, 2.0);
        EXPECT_DOUBLE_EQ(markers[0].pose.position.z, 0.5);
        EXPECT_FLOAT_EQ(markers[0].color.r, 0.0f);
        EXPECT_FLOAT_EQ(markers[0].color.g, 0.0f);
        EXPECT_FLOAT_EQ(markers[0].color.b, 1.0f); // Blue for our team

        // Check second robot (theirs - red)
        EXPECT_EQ(markers[1].ns, "robots");
        EXPECT_EQ(markers[1].id, 1);
        EXPECT_DOUBLE_EQ(markers[1].pose.position.x, -1.0);
        EXPECT_DOUBLE_EQ(markers[1].pose.position.y, -2.0);
        EXPECT_FLOAT_EQ(markers[1].color.r, 1.0f); // Red for opponent
        EXPECT_FLOAT_EQ(markers[1].color.g, 0.0f);
        EXPECT_FLOAT_EQ(markers[1].color.b, 0.0f);
    }

    // Test empty keypoints
    TEST_F(RosMarkerAdapterTest, EmptyKeypoints)
    {
        KeypointsStamped empty_keypoints;
        empty_keypoints.header = test_header;

        auto markers = ros_adapters::to_ros_keypoint_markers(empty_keypoints);

        EXPECT_EQ(markers.size(), 0);
    }

    // Test empty robots
    TEST_F(RosMarkerAdapterTest, EmptyRobots)
    {
        RobotDescriptionsStamped empty_robots;
        empty_robots.header = test_header;

        auto markers = ros_adapters::to_ros_robot_markers(empty_robots);

        EXPECT_EQ(markers.size(), 0);
    }

    // Test robot size
    TEST_F(RosMarkerAdapterTest, RobotSize)
    {
        auto markers = ros_adapters::to_ros_robot_markers(robots);

        EXPECT_DOUBLE_EQ(markers[0].scale.x, 0.3);
        EXPECT_DOUBLE_EQ(markers[0].scale.y, 0.3);
        EXPECT_DOUBLE_EQ(markers[0].scale.z, 0.2);
    }

    // Test header propagation
    TEST_F(RosMarkerAdapterTest, HeaderPropagation)
    {
        auto field_marker = ros_adapters::to_ros_field_marker(field_desc);
        EXPECT_DOUBLE_EQ(field_marker.header.stamp.toSec(), 123.456);
        EXPECT_EQ(field_marker.header.frame_id, "camera");

        auto keypoint_markers = ros_adapters::to_ros_keypoint_markers(keypoints);
        EXPECT_DOUBLE_EQ(keypoint_markers[0].header.stamp.toSec(), 123.456);

        auto robot_markers = ros_adapters::to_ros_robot_markers(robots);
        EXPECT_DOUBLE_EQ(robot_markers[0].header.stamp.toSec(), 123.456);
    }

} // namespace auto_battlebot
