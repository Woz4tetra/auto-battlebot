#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "keypoint_model/config.hpp"
#include "keypoint_model/sim_keypoint_model.hpp"
#include "robot_filter/front_back_keypoint_converter.hpp"

namespace auto_battlebot {
namespace {

/** camera_view_matrix() from simulation/camera_utils.py: camera-from-world, z forward, y down. */
Eigen::Matrix4d camera_from_field(const Eigen::Vector3d &position, const Eigen::Vector3d &lookat) {
    const Eigen::Vector3d forward = (lookat - position).normalized();
    const Eigen::Vector3d right = forward.cross(Eigen::Vector3d::UnitZ()).normalized();
    const Eigen::Vector3d up = right.cross(forward);
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf.block<1, 3>(0, 0) = right.transpose();
    tf.block<1, 3>(1, 0) = -up.transpose();
    tf.block<1, 3>(2, 0) = forward.transpose();
    tf.block<3, 1>(0, 3) = -tf.block<3, 3>(0, 0) * position;
    return tf;
}

CameraInfo camera_info() {
    CameraInfo info;
    info.width = 640;
    info.height = 360;
    info.intrinsics = cv::Mat(cv::Matx33d(650.0, 0.0, 320.0, 0.0, 650.0, 180.0, 0.0, 0.0, 1.0));
    info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    return info;
}

class SimKeypointModelTest : public ::testing::Test {
   protected:
    Eigen::Matrix4d tf = camera_from_field({0.0, -0.5, 2.6}, {0.0, 0.0, 0.0});
    CameraInfo info = camera_info();
    Header image_header{2.0, FrameId::CAMERA};
    std::vector<Label> labels{Label::MR_STABS_MK2, Label::OPPONENT, Label::HOUSE_BOT};

    SimMeasurementFrame frame() const {
        SimMeasurementFrame f;
        f.delivered = true;
        f.capture_stamp = 1.9;
        f.robots = {
            {true, Pose2D{0.3, 0.2, 0.7}},
            {true, Pose2D{-0.4, 0.5, -2.0}},
            {false, Pose2D{0.0, 0.0, 0.0}},
        };
        return f;
    }
};

TEST_F(SimKeypointModelTest, MeasuredRobotsBecomeFrontBackPairsAtCaptureStamp) {
    const ModelResultStamped result =
        sim_measurements_to_keypoints(frame(), labels, 0.05, tf, info, image_header);
    EXPECT_DOUBLE_EQ(result.header.stamp, 1.9);
    ASSERT_EQ(result.keypoints.size(), 4u);
    EXPECT_EQ(result.keypoints[0].keypoint_label, KeypointLabel::MR_STABS_MK2_FRONT);
    EXPECT_EQ(result.keypoints[1].keypoint_label, KeypointLabel::MR_STABS_MK2_BACK);
    EXPECT_EQ(result.keypoints[2].keypoint_label, KeypointLabel::OPPONENT_FRONT);
    EXPECT_EQ(result.keypoints[3].keypoint_label, KeypointLabel::OPPONENT_BACK);
    EXPECT_EQ(result.keypoints[2].detection_index, 1);
    EXPECT_TRUE(result.boxes.empty());
}

TEST_F(SimKeypointModelTest, UndeliveredFrameIsEmptyAtImageStamp) {
    SimMeasurementFrame f = frame();
    f.delivered = false;
    const ModelResultStamped result =
        sim_measurements_to_keypoints(f, labels, 0.05, tf, info, image_header);
    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_DOUBLE_EQ(result.header.stamp, 2.0);
}

TEST_F(SimKeypointModelTest, OffImageRobotIsDropped) {
    SimMeasurementFrame f = frame();
    f.robots[0].pose = Pose2D{25.0, 0.0, 0.0};
    const ModelResultStamped result =
        sim_measurements_to_keypoints(f, labels, 0.05, tf, info, image_header);
    ASSERT_EQ(result.keypoints.size(), 2u);
    EXPECT_EQ(result.keypoints[0].label, Label::OPPONENT);
}

TEST_F(SimKeypointModelTest, ConverterRecoversTheMeasuredPose) {
    const SimMeasurementFrame f = frame();
    const ModelResultStamped result =
        sim_measurements_to_keypoints(f, labels, 0.05, tf, info, image_header);

    FrontBackKeypointConverterConfig converter_config;
    converter_config.front_keypoints = {KeypointLabel::MR_STABS_MK2_FRONT,
                                        KeypointLabel::OPPONENT_FRONT};
    converter_config.back_keypoints = {KeypointLabel::MR_STABS_MK2_BACK,
                                       KeypointLabel::OPPONENT_BACK};
    // A configured height the sim keypoints must not use: they carry their own (zero).
    converter_config.keypoint_heights.default_meters = 0.03;
    FrontBackKeypointConverter converter(converter_config);

    FieldDescription field;
    field.tf_camera_from_fieldcenter.tf = tf;
    const auto converted = converter.convert(result, field, info);
    const Eigen::Matrix4d field_from_camera = tf.inverse();

    for (size_t slot = 0; slot < 2; ++slot) {
        const auto it = converted.find(labels[slot]);
        ASSERT_NE(it, converted.end());
        ASSERT_EQ(it->second.size(), 1u);
        const FrontBackAssignment &assignment = it->second[0].first;
        const Eigen::Vector3d front =
            (field_from_camera * assignment.front.homogeneous()).head<3>();
        const Eigen::Vector3d back = (field_from_camera * assignment.back.homogeneous()).head<3>();
        const Pose2D &pose = f.robots[slot].pose;
        EXPECT_NEAR(0.5 * (front.x() + back.x()), pose.x, 1e-6);
        EXPECT_NEAR(0.5 * (front.y() + back.y()), pose.y, 1e-6);
        EXPECT_NEAR(std::atan2(front.y() - back.y(), front.x() - back.x()), pose.yaw, 1e-6);
    }
}

TEST(SimKeypointModelConfigTest, LabelWithoutKeypointsListsValidLabels) {
    toml::table table{{"labels", toml::array{"NOT_A_LABEL"}}};
    ConfigParser parser(table, "keypoint_model");
    SimKeypointModelConfiguration config;
    try {
        config.parse_fields(parser);
        FAIL() << "expected a parse error";
    } catch (const std::invalid_argument &error) {
        EXPECT_NE(std::string(error.what()).find("MR_STABS_MK2"), std::string::npos);
    }
}

}  // namespace
}  // namespace auto_battlebot
