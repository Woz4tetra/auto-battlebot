#include <gtest/gtest.h>

#include <visualization_msgs/Marker.hxx>

#include "ros/ros_message_adapters/ros_marker.hpp"

namespace auto_battlebot {
class RosMarkerAdapterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        test_header.stamp = 123.456;
        test_header.frame_id = FrameId::CAMERA;

        // Setup field description
        field_desc.header = test_header;
        field_desc.size.header = test_header;
        field_desc.size.size.x = 2.0;
        field_desc.size.size.y = 3.0;
        field_desc.size.size.z = 0.1;

        // Create transform for field
        field_desc.tf_camera_from_fieldcenter.tf = Eigen::MatrixXd::Identity(4, 4);
        field_desc.tf_camera_from_fieldcenter.tf(0, 3) = 1.0;
        field_desc.tf_camera_from_fieldcenter.tf(1, 3) = 2.0;
        field_desc.tf_camera_from_fieldcenter.tf(2, 3) = 3.0;

        // Setup keypoints
        keypoints.header = test_header;
        Keypoint kp1;
        kp1.label = Label::MR_STABS_MK1;
        kp1.keypoint_label = KeypointLabel::MR_STABS_MK1_FRONT;
        kp1.x = 10.0f;
        kp1.y = 20.0f;
        keypoints.keypoints.push_back(kp1);

        Keypoint kp2;
        kp2.label = Label::MR_STABS_MK2;
        kp2.keypoint_label = KeypointLabel::MR_STABS_MK2_FRONT;
        kp2.x = 30.0f;
        kp2.y = 40.0f;
        keypoints.keypoints.push_back(kp2);

        // Setup robots
        robots.header = test_header;
        RobotDescription robot1;
        robot1.label = Label::MR_STABS_MK1;
        robot1.frame_id = FrameId::FIELD;
        robot1.pose.position.x = 1.0;
        robot1.pose.position.y = 2.0;
        robot1.pose.position.z = 0.5;
        robot1.pose.rotation.w = 1.0;
        robot1.size.x = 0.3;
        robot1.size.y = 0.3;
        robot1.size.z = 0.2;
        robots.descriptions.push_back(robot1);

        RobotDescription robot2;
        robot2.label = Label::MR_STABS_MK2;
        robot2.frame_id = FrameId::FIELD;
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
    FieldDescriptionWithInlierPoints field_desc;
    KeypointsStamped keypoints;
    RobotDescriptionsStamped robots;
};

// Test field marker conversion
TEST_F(RosMarkerAdapterTest, FieldMarkerConversion) {
    auto markers = ros_adapters::to_ros_field_marker(field_desc);

    ASSERT_FALSE(markers.empty());
    auto &marker = markers[0];

    EXPECT_EQ(marker.ns, "field");
    EXPECT_EQ(marker.id, 0);
    EXPECT_EQ(marker.type, visualization_msgs::Marker::LINE_STRIP);
    EXPECT_EQ(marker.action, visualization_msgs::Marker::ADD);

    // Pose is identity (corners are pre-transformed into the points list)
    EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.0);

    // Line width
    EXPECT_DOUBLE_EQ(marker.scale.x, 0.01);

    // 4 corners + 1 closing point
    EXPECT_EQ(marker.points.size(), 5u);

    // Check color (green border, fully opaque)
    EXPECT_FLOAT_EQ(marker.color.r, 0.0f);
    EXPECT_FLOAT_EQ(marker.color.g, 1.0f);
    EXPECT_FLOAT_EQ(marker.color.b, 0.0f);
    EXPECT_FLOAT_EQ(marker.color.a, 1.0f);
}

// Test robot markers conversion
// Each robot generates 3 markers: cube (bounds), arrow (pose), text (label)
TEST_F(RosMarkerAdapterTest, RobotMarkersConversion) {
    auto markers = ros_adapters::to_ros_robot_markers(robots);

    // 2 robots * 3 markers each = 6 markers
    EXPECT_EQ(markers.size(), 6);

    // IDs come from magic_enum::enum_index(robot.frame_id); both robots have FrameId::FIELD = 11
    constexpr int kFieldFrameId = 11;

    // Check first robot's cube marker (bounds)
    EXPECT_EQ(markers[0].ns, "robot_bounds");
    EXPECT_EQ(markers[0].id, kFieldFrameId);
    EXPECT_EQ(markers[0].type, visualization_msgs::Marker::CUBE);
    EXPECT_DOUBLE_EQ(markers[0].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(markers[0].pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(markers[0].pose.position.z, 0.5);
    // Color is determined by label via get_color_for_index()
    EXPECT_GT(markers[0].color.a, 0.0f);  // Has some alpha

    // Check first robot's arrow marker (pose)
    EXPECT_EQ(markers[1].ns, "robot_poses");
    EXPECT_EQ(markers[1].id, kFieldFrameId);
    EXPECT_EQ(markers[1].type, visualization_msgs::Marker::ARROW);
    EXPECT_DOUBLE_EQ(markers[1].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(markers[1].pose.position.y, 2.0);

    // Check first robot's text marker (label)
    EXPECT_EQ(markers[2].ns, "robot_labels");
    EXPECT_EQ(markers[2].id, kFieldFrameId);
    EXPECT_EQ(markers[2].type, visualization_msgs::Marker::TEXT_VIEW_FACING);

    // Check second robot's cube marker (bounds)
    EXPECT_EQ(markers[3].ns, "robot_bounds");
    EXPECT_EQ(markers[3].id, kFieldFrameId);
    EXPECT_EQ(markers[3].type, visualization_msgs::Marker::CUBE);
    EXPECT_DOUBLE_EQ(markers[3].pose.position.x, -1.0);
    EXPECT_DOUBLE_EQ(markers[3].pose.position.y, -2.0);

    // Check second robot's arrow marker (pose)
    EXPECT_EQ(markers[4].ns, "robot_poses");
    EXPECT_EQ(markers[4].id, kFieldFrameId);
    EXPECT_EQ(markers[4].type, visualization_msgs::Marker::ARROW);

    // Check second robot's text marker (label)
    EXPECT_EQ(markers[5].ns, "robot_labels");
    EXPECT_EQ(markers[5].id, kFieldFrameId);
    EXPECT_EQ(markers[5].type, visualization_msgs::Marker::TEXT_VIEW_FACING);
}

// Test empty robots
TEST_F(RosMarkerAdapterTest, EmptyRobots) {
    RobotDescriptionsStamped empty_robots;
    empty_robots.header = test_header;

    auto markers = ros_adapters::to_ros_robot_markers(empty_robots);

    EXPECT_EQ(markers.size(), 0);
}

// Test robot size
TEST_F(RosMarkerAdapterTest, RobotSize) {
    auto markers = ros_adapters::to_ros_robot_markers(robots);

    EXPECT_DOUBLE_EQ(markers[0].scale.x, 0.3);
    EXPECT_DOUBLE_EQ(markers[0].scale.y, 0.3);
    EXPECT_DOUBLE_EQ(markers[0].scale.z, 0.2);
}

// Test header propagation
TEST_F(RosMarkerAdapterTest, HeaderPropagation) {
    auto field_markers = ros_adapters::to_ros_field_marker(field_desc);
    ASSERT_FALSE(field_markers.empty());
    EXPECT_DOUBLE_EQ(field_markers[0].header.stamp.toSec(), 123.456);
    EXPECT_EQ(field_markers[0].header.frame_id, "camera");

    auto robot_markers = ros_adapters::to_ros_robot_markers(robots);
    EXPECT_DOUBLE_EQ(robot_markers[0].header.stamp.toSec(), 123.456);
}

}  // namespace auto_battlebot
