#include <gtest/gtest.h>
#include "ros/ros_message_adapters/ros_tf2.hpp"
#include <tf2_msgs/TFMessage.hxx>
#include <cmath>

namespace auto_battlebot
{
    class RosTF2AdapterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            test_header.timestamp = 123.456;
            test_header.frame_id = FrameId::CAMERA;

            // Create identity transform
            identity_transform.header = test_header;
            identity_transform.child_frame_id = FrameId::FIELD;
            identity_transform.transform.tf = Eigen::MatrixXd::Identity(4, 4);

            // Create translation transform
            translation_transform.header = test_header;
            translation_transform.child_frame_id = FrameId::FIELD;
            translation_transform.transform.tf = Eigen::MatrixXd::Identity(4, 4);
            translation_transform.transform.tf(0, 3) = 1.0;
            translation_transform.transform.tf(1, 3) = 2.0;
            translation_transform.transform.tf(2, 3) = 3.0;

            // Create rotation transform (90 degrees around Z-axis)
            rotation_transform.header = test_header;
            rotation_transform.child_frame_id = FrameId::FIELD;
            rotation_transform.transform.tf = Eigen::MatrixXd::Identity(4, 4);
            rotation_transform.transform.tf(0, 0) = 0.0;
            rotation_transform.transform.tf(0, 1) = -1.0;
            rotation_transform.transform.tf(1, 0) = 1.0;
            rotation_transform.transform.tf(1, 1) = 0.0;
        }

        Header test_header;
        TransformStamped identity_transform;
        TransformStamped translation_transform;
        TransformStamped rotation_transform;
    };

    // Test identity transform conversion
    TEST_F(RosTF2AdapterTest, IdentityTransform)
    {
        auto tf_stamped = ros_adapters::to_ros_transform_stamped(identity_transform);

        // Check header
        EXPECT_DOUBLE_EQ(tf_stamped.header.stamp.toSec(), 123.456);
        EXPECT_EQ(tf_stamped.header.frame_id, "camera");
        EXPECT_EQ(tf_stamped.child_frame_id, "field");

        // Check translation
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.x, 0.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.y, 0.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.z, 0.0);

        // Check rotation (identity quaternion)
        EXPECT_DOUBLE_EQ(tf_stamped.transform.rotation.x, 0.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.rotation.y, 0.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.rotation.z, 0.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.rotation.w, 1.0);
    }

    // Test translation transform
    TEST_F(RosTF2AdapterTest, TranslationTransform)
    {
        auto tf_stamped = ros_adapters::to_ros_transform_stamped(translation_transform);

        // Check translation
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.x, 1.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.y, 2.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.z, 3.0);

        // Check rotation (still identity)
        EXPECT_DOUBLE_EQ(tf_stamped.transform.rotation.w, 1.0);
    }

    // Test rotation transform
    TEST_F(RosTF2AdapterTest, RotationTransform)
    {
        auto tf_stamped = ros_adapters::to_ros_transform_stamped(rotation_transform);

        // For 90 degree rotation around Z-axis, quaternion should be:
        // w = cos(45°), z = sin(45°), x = y = 0
        EXPECT_NEAR(tf_stamped.transform.rotation.x, 0.0, 1e-6);
        EXPECT_NEAR(tf_stamped.transform.rotation.y, 0.0, 1e-6);
        EXPECT_NEAR(tf_stamped.transform.rotation.z, 0.7071067811865475, 1e-6);
        EXPECT_NEAR(tf_stamped.transform.rotation.w, 0.7071067811865475, 1e-6);
    }

    // Test quaternion normalization
    TEST_F(RosTF2AdapterTest, QuaternionNormalized)
    {
        auto tf_stamped = ros_adapters::to_ros_transform_stamped(rotation_transform);

        // Quaternion should be normalized
        double magnitude = std::sqrt(
            tf_stamped.transform.rotation.x * tf_stamped.transform.rotation.x +
            tf_stamped.transform.rotation.y * tf_stamped.transform.rotation.y +
            tf_stamped.transform.rotation.z * tf_stamped.transform.rotation.z +
            tf_stamped.transform.rotation.w * tf_stamped.transform.rotation.w);

        EXPECT_NEAR(magnitude, 1.0, 1e-6);
    }

    // Test TFMessage creation
    TEST_F(RosTF2AdapterTest, TFMessageSingleTransform)
    {
        auto tf_msg = ros_adapters::to_ros_tf_message(identity_transform);

        EXPECT_EQ(tf_msg.transforms.size(), 1);
        EXPECT_EQ(tf_msg.transforms[0].header.frame_id, "camera");
        EXPECT_EQ(tf_msg.transforms[0].child_frame_id, "field");
    }

    // Test combined rotation and translation
    TEST_F(RosTF2AdapterTest, CombinedRotationAndTranslation)
    {
        TransformStamped combined;
        combined.header = test_header;
        combined.child_frame_id = FrameId::FIELD;
        combined.transform.tf = Eigen::MatrixXd::Identity(4, 4);

        // 90 degree rotation around Z
        combined.transform.tf(0, 0) = 0.0;
        combined.transform.tf(0, 1) = -1.0;
        combined.transform.tf(1, 0) = 1.0;
        combined.transform.tf(1, 1) = 0.0;

        // Plus translation
        combined.transform.tf(0, 3) = 5.0;
        combined.transform.tf(1, 3) = 6.0;
        combined.transform.tf(2, 3) = 7.0;

        auto tf_stamped = ros_adapters::to_ros_transform_stamped(combined);

        // Check translation
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.x, 5.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.y, 6.0);
        EXPECT_DOUBLE_EQ(tf_stamped.transform.translation.z, 7.0);

        // Check rotation
        EXPECT_NEAR(tf_stamped.transform.rotation.z, 0.7071067811865475, 1e-6);
        EXPECT_NEAR(tf_stamped.transform.rotation.w, 0.7071067811865475, 1e-6);
    }

} // namespace auto_battlebot
