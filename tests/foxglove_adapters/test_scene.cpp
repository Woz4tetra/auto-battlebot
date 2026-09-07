#include <gtest/gtest.h>

#include <cstring>

#include "foxglove_adapters/scene.hpp"

namespace auto_battlebot {
class SceneAdapterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        test_header.stamp = 123.456;
        test_header.frame_id = FrameId::CAMERA;

        field_desc.header = test_header;
        field_desc.size.header = test_header;
        field_desc.size.size.x = 2.0;
        field_desc.size.size.y = 3.0;
        field_desc.size.size.z = 0.1;
        field_desc.tf_camera_from_fieldcenter.tf = Eigen::MatrixXd::Identity(4, 4);
        field_desc.tf_camera_from_fieldcenter.tf(0, 3) = 1.0;
        field_desc.tf_camera_from_fieldcenter.tf(1, 3) = 2.0;
        field_desc.tf_camera_from_fieldcenter.tf(2, 3) = 3.0;

        robots.header = test_header;
        RobotDescription robot1;
        robot1.label = Label::MR_STABS_MK1;
        robot1.frame_id = FrameId::OUR_ROBOT_1;
        robot1.pose.position.x = 1.0;
        robot1.pose.position.y = 2.0;
        robot1.pose.position.z = 0.5;
        robot1.pose.rotation.w = 1.0;
        robot1.size.x = 0.3;
        robot1.size.y = 0.3;
        robot1.size.z = 0.2;
        robots.descriptions.push_back(robot1);

        RobotDescription robot2 = robot1;
        robot2.frame_id = FrameId::THEIR_ROBOT_1;
        robot2.group = Group::THEIRS;
        robot2.pose.position.x = -1.0;
        robot2.pose.position.y = -2.0;
        robot2.keypoints.push_back({0.0, 0.0, 0.0});
        robot2.keypoints.push_back({0.1, 0.0, 0.0});
        robots.descriptions.push_back(robot2);
    }

    Header test_header;
    FieldDescriptionWithInlierPoints field_desc;
    RobotDescriptionsStamped robots;
};

TEST_F(SceneAdapterTest, FieldBorderIsClosedLineStrip) {
    auto update = foxglove_adapters::to_field_scene(field_desc);

    ASSERT_EQ(update.entities.size(), 1u);
    const auto &entity = update.entities[0];
    EXPECT_EQ(entity.id, "field/0");
    EXPECT_EQ(entity.frame_id, "camera");
    ASSERT_EQ(entity.lines.size(), 1u);
    const auto &line = entity.lines[0];
    EXPECT_EQ(line.type, foxglove::schemas::LinePrimitive::LineType::LINE_STRIP);
    EXPECT_DOUBLE_EQ(line.thickness, 0.01);
    ASSERT_EQ(line.points.size(), 5u);
    // First corner (-1, -1.5, 0) translated by (1, 2, 3)
    EXPECT_DOUBLE_EQ(line.points[0].x, 0.0);
    EXPECT_DOUBLE_EQ(line.points[0].y, 0.5);
    EXPECT_DOUBLE_EQ(line.points[0].z, 3.0);
    EXPECT_DOUBLE_EQ(line.points[4].x, line.points[0].x);
    ASSERT_TRUE(line.color.has_value());
    EXPECT_DOUBLE_EQ(line.color->g, 1.0);
}

TEST_F(SceneAdapterTest, PointCloudIsPackedFloat32Xyz) {
    field_desc.inlier_points.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    field_desc.inlier_points.cloud->points.push_back(pcl::PointXYZ(1.5f, -2.25f, 0.125f));
    field_desc.inlier_points.cloud->points.push_back(pcl::PointXYZ(0.0f, 1.0f, 2.0f));

    auto cloud = foxglove_adapters::to_field_point_cloud(field_desc);
    ASSERT_TRUE(cloud.has_value());
    EXPECT_EQ(cloud->point_stride, 12u);
    ASSERT_EQ(cloud->fields.size(), 3u);
    EXPECT_EQ(cloud->fields[2].name, "z");
    EXPECT_EQ(cloud->fields[2].offset, 8u);
    EXPECT_EQ(cloud->fields[2].type, foxglove::schemas::PackedElementField::NumericType::FLOAT32);
    ASSERT_EQ(cloud->data.size(), 24u);
    float xyz[3];
    std::memcpy(xyz, cloud->data.data(), 12);
    EXPECT_FLOAT_EQ(xyz[0], 1.5f);
    EXPECT_FLOAT_EQ(xyz[1], -2.25f);
    EXPECT_FLOAT_EQ(xyz[2], 0.125f);
    EXPECT_EQ(cloud->frame_id, "camera");
}

TEST_F(SceneAdapterTest, NoCloudNoPointCloud) {
    EXPECT_FALSE(foxglove_adapters::to_field_point_cloud(field_desc).has_value());
}

TEST_F(SceneAdapterTest, RobotEntitiesUseFrameIndexIds) {
    auto update = foxglove_adapters::to_robot_scene(robots);

    // robot1: cube, arrow, text. robot2: cube, arrow, text, 2 keypoints, keypoint line.
    ASSERT_EQ(update.entities.size(), 9u);
    EXPECT_EQ(update.entities[0].id, "robot_bounds/4");
    ASSERT_EQ(update.entities[0].cubes.size(), 1u);
    EXPECT_DOUBLE_EQ(update.entities[0].cubes[0].pose->position->x, 1.0);
    EXPECT_DOUBLE_EQ(update.entities[0].cubes[0].size->x, 0.3);
    EXPECT_EQ(update.entities[1].id, "robot_poses/4");
    ASSERT_EQ(update.entities[1].arrows.size(), 1u);
    EXPECT_NEAR(
        update.entities[1].arrows[0].shaft_length + update.entities[1].arrows[0].head_length, 0.45,
        1e-9);
    EXPECT_EQ(update.entities[2].id, "robot_labels/4");
    ASSERT_EQ(update.entities[2].texts.size(), 1u);
    EXPECT_TRUE(update.entities[2].texts[0].billboard);
    EXPECT_EQ(update.entities[2].texts[0].text, "our_robot_1 (mr_stabs_mk1)");

    EXPECT_EQ(update.entities[3].id, "robot_bounds/6");
    EXPECT_EQ(update.entities[6].id, "robot_keypoints/0");
    EXPECT_EQ(update.entities[7].id, "robot_keypoints/1");
    EXPECT_EQ(update.entities[8].id, "robot_keypoint_lines/6");
    ASSERT_EQ(update.entities[8].lines.size(), 1u);
    EXPECT_EQ(update.entities[8].lines[0].points.size(), 2u);
}

TEST_F(SceneAdapterTest, EmptyRobotsStillClearsTheScene) {
    RobotDescriptionsStamped empty;
    empty.header = test_header;
    auto update = foxglove_adapters::to_robot_scene(empty);
    EXPECT_TRUE(update.entities.empty());
    ASSERT_EQ(update.deletions.size(), 1u);
    EXPECT_EQ(update.deletions[0].type,
              foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL);
}

TEST_F(SceneAdapterTest, EmptyHazardsStillEncode) {
    field_desc.hazards.clear();
    auto update = foxglove_adapters::to_hazard_scene(field_desc);
    std::vector<uint8_t> buffer(256);
    size_t encoded = 0;
    ASSERT_EQ(update.encode(buffer.data(), buffer.size(), &encoded), foxglove::FoxgloveError::Ok);
    EXPECT_GT(encoded, 0u);
}

TEST_F(SceneAdapterTest, HazardRings) {
    FieldHazard hazard;
    hazard.center.x = 0.5;
    hazard.center.y = 0.0;
    hazard.object_radius = 0.1;
    hazard.source = HazardSource::STATIC;
    field_desc.hazards.push_back(hazard);

    auto update = foxglove_adapters::to_hazard_scene(field_desc);
    ASSERT_EQ(update.deletions.size(), 1u);
    EXPECT_EQ(update.deletions[0].type,
              foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL);
    ASSERT_EQ(update.entities.size(), 1u);
    EXPECT_EQ(update.entities[0].id, "hazards/0");
    ASSERT_EQ(update.entities[0].lines.size(), 1u);
    EXPECT_EQ(update.entities[0].lines[0].points.size(), 49u);
    EXPECT_NEAR(update.entities[0].lines[0].color->g, 0.65, 1e-6);
}

TEST_F(SceneAdapterTest, NavigationDeletesAbsentNamespaces) {
    NavigationVisualization nav;
    nav.header = test_header;
    auto update = foxglove_adapters::to_navigation_scene(nav);
    EXPECT_TRUE(update.entities.empty());
    ASSERT_EQ(update.deletions.size(), 5u);
    EXPECT_EQ(update.deletions[0].id, "nav_pursuit_line/0");
    EXPECT_EQ(update.deletions[0].type,
              foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID);
}

TEST_F(SceneAdapterTest, NavigationDrawsPathAndVelocity) {
    NavigationVisualization nav;
    nav.header = test_header;
    nav.path = NavigationPathSegment{};
    nav.path->our_x = 0.0;
    nav.path->our_y = 0.0;
    nav.path->target_x = 1.0;
    nav.path->target_y = 0.0;
    nav.command.linear_x = 1.0;
    nav.command.angular_z = 1.0;
    nav.robots = robots;

    auto update = foxglove_adapters::to_navigation_scene(nav);
    ASSERT_EQ(update.entities.size(), 5u);
    EXPECT_EQ(update.entities[0].id, "nav_pursuit_line/0");
    EXPECT_EQ(update.entities[1].id, "nav_target/1");
    EXPECT_EQ(update.entities[2].id, "nav_velocity/2");
    ASSERT_EQ(update.entities[2].arrows.size(), 1u);
    ASSERT_TRUE(update.entities[2].lifetime.has_value());
    EXPECT_EQ(update.entities[2].lifetime->nsec, 100000000u);
    // Velocity arrow points along +x from the robot at (1, 2)
    EXPECT_NEAR(update.entities[2].arrows[0].pose->orientation->w, 1.0, 1e-9);
    EXPECT_NEAR(
        update.entities[2].arrows[0].shaft_length + update.entities[2].arrows[0].head_length, 0.15,
        1e-9);
    EXPECT_EQ(update.entities[3].id, "nav_angular/3");
    EXPECT_EQ(update.entities[4].id, "nav_angular_head/4");
    EXPECT_TRUE(update.deletions.empty());
}

}  // namespace auto_battlebot
