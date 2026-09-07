#include <gtest/gtest.h>

#include <cmath>

#include "foxglove_adapters/transform.hpp"

namespace auto_battlebot {
class FrameTransformAdapterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        test_header.stamp = 123.456;
        test_header.frame_id = FrameId::CAMERA;

        identity_transform.header = test_header;
        identity_transform.child_frame_id = FrameId::FIELD;
        identity_transform.transform.tf = Eigen::MatrixXd::Identity(4, 4);

        rotation_transform = identity_transform;
        rotation_transform.transform.tf(0, 0) = 0.0;
        rotation_transform.transform.tf(0, 1) = -1.0;
        rotation_transform.transform.tf(1, 0) = 1.0;
        rotation_transform.transform.tf(1, 1) = 0.0;
        rotation_transform.transform.tf(0, 3) = 5.0;
        rotation_transform.transform.tf(1, 3) = 6.0;
        rotation_transform.transform.tf(2, 3) = 7.0;
    }

    Header test_header;
    TransformStamped identity_transform;
    TransformStamped rotation_transform;
};

TEST_F(FrameTransformAdapterTest, IdentityTransform) {
    auto tf = foxglove_adapters::to_frame_transform(identity_transform);

    ASSERT_TRUE(tf.timestamp.has_value());
    EXPECT_EQ(tf.timestamp->sec, 123u);
    EXPECT_EQ(tf.parent_frame_id, "camera");
    EXPECT_EQ(tf.child_frame_id, "field");
    ASSERT_TRUE(tf.translation.has_value());
    EXPECT_DOUBLE_EQ(tf.translation->x, 0.0);
    ASSERT_TRUE(tf.rotation.has_value());
    EXPECT_DOUBLE_EQ(tf.rotation->w, 1.0);
    EXPECT_DOUBLE_EQ(tf.rotation->z, 0.0);
}

TEST_F(FrameTransformAdapterTest, RotationAndTranslation) {
    auto tf = foxglove_adapters::to_frame_transform(rotation_transform);

    EXPECT_DOUBLE_EQ(tf.translation->x, 5.0);
    EXPECT_DOUBLE_EQ(tf.translation->y, 6.0);
    EXPECT_DOUBLE_EQ(tf.translation->z, 7.0);
    // 90 degrees about Z
    EXPECT_NEAR(tf.rotation->w, std::sqrt(0.5), 1e-6);
    EXPECT_NEAR(std::abs(tf.rotation->z), std::sqrt(0.5), 1e-6);
    EXPECT_NEAR(tf.rotation->x, 0.0, 1e-6);
    EXPECT_NEAR(tf.rotation->y, 0.0, 1e-6);
}

TEST_F(FrameTransformAdapterTest, DegenerateMatrixFallsBackToIdentity) {
    TransformStamped bad = identity_transform;
    bad.transform.tf = Eigen::MatrixXd::Identity(2, 2);
    auto tf = foxglove_adapters::to_frame_transform(bad);
    EXPECT_DOUBLE_EQ(tf.rotation->w, 1.0);
    EXPECT_DOUBLE_EQ(tf.translation->x, 0.0);
}

TEST_F(FrameTransformAdapterTest, WrapsSingleTransform) {
    auto msg = foxglove_adapters::to_frame_transforms(identity_transform);
    ASSERT_EQ(msg.transforms.size(), 1u);
    EXPECT_EQ(msg.transforms[0].child_frame_id, "field");
}

}  // namespace auto_battlebot
