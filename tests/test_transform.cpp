#include <gtest/gtest.h>
#include "data_structures/transform.hpp"

namespace auto_battlebot {

TEST(TransformTest, DefaultConstruction) {
    Transform tf;
    // Eigen::MatrixXd should be constructible
    EXPECT_EQ(tf.tf.size(), 0);
}

TEST(TransformTest, IdentityMatrix) {
    Transform tf;
    tf.tf = Eigen::MatrixXd::Identity(4, 4);
    
    EXPECT_EQ(tf.tf.rows(), 4);
    EXPECT_EQ(tf.tf.cols(), 4);
    
    EXPECT_DOUBLE_EQ(tf.tf(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(tf.tf(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(tf.tf(2, 2), 1.0);
    EXPECT_DOUBLE_EQ(tf.tf(3, 3), 1.0);
    
    EXPECT_DOUBLE_EQ(tf.tf(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(tf.tf(1, 0), 0.0);
}

TEST(TransformTest, TranslationMatrix) {
    Transform tf;
    tf.tf = Eigen::MatrixXd::Identity(4, 4);
    tf.tf(0, 3) = 1.0;  // x translation
    tf.tf(1, 3) = 2.0;  // y translation
    tf.tf(2, 3) = 3.0;  // z translation
    
    EXPECT_DOUBLE_EQ(tf.tf(0, 3), 1.0);
    EXPECT_DOUBLE_EQ(tf.tf(1, 3), 2.0);
    EXPECT_DOUBLE_EQ(tf.tf(2, 3), 3.0);
}

TEST(TransformStampedTest, DefaultConstruction) {
    TransformStamped tf_stamped;
    
    EXPECT_EQ(tf_stamped.header.timestamp, 0.0);
    EXPECT_EQ(tf_stamped.header.frame_id, "");
    EXPECT_EQ(tf_stamped.child_frame_id, "");
    EXPECT_EQ(tf_stamped.transform.tf.size(), 0);
}

TEST(TransformStampedTest, ValueAssignment) {
    TransformStamped tf_stamped;
    
    tf_stamped.header.timestamp = 123.456;
    tf_stamped.header.frame_id = "world";
    tf_stamped.child_frame_id = "robot";
    tf_stamped.transform.tf = Eigen::MatrixXd::Identity(4, 4);
    
    EXPECT_DOUBLE_EQ(tf_stamped.header.timestamp, 123.456);
    EXPECT_EQ(tf_stamped.header.frame_id, "world");
    EXPECT_EQ(tf_stamped.child_frame_id, "robot");
    EXPECT_EQ(tf_stamped.transform.tf.rows(), 4);
    EXPECT_EQ(tf_stamped.transform.tf.cols(), 4);
}

}  // namespace auto_battlebot
