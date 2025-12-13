#include <gtest/gtest.h>
#include <cmath>
#include "data_structures/pose.hpp"

namespace auto_battlebot {

TEST(PositionTest, DefaultConstruction) {
    Position pos;
    EXPECT_EQ(pos.x, 0.0);
    EXPECT_EQ(pos.y, 0.0);
    EXPECT_EQ(pos.z, 0.0);
}

TEST(PositionTest, ValueAssignment) {
    Position pos;
    pos.x = 1.0;
    pos.y = 2.0;
    pos.z = 3.0;
    
    EXPECT_DOUBLE_EQ(pos.x, 1.0);
    EXPECT_DOUBLE_EQ(pos.y, 2.0);
    EXPECT_DOUBLE_EQ(pos.z, 3.0);
}

TEST(RotationTest, DefaultConstruction) {
    Rotation rot;
    EXPECT_EQ(rot.w, 0.0);
    EXPECT_EQ(rot.x, 0.0);
    EXPECT_EQ(rot.y, 0.0);
    EXPECT_EQ(rot.z, 0.0);
}

TEST(RotationTest, ValueAssignment) {
    Rotation rot;
    rot.w = 1.0;
    rot.x = 0.0;
    rot.y = 0.0;
    rot.z = 0.0;
    
    EXPECT_DOUBLE_EQ(rot.w, 1.0);
    EXPECT_DOUBLE_EQ(rot.x, 0.0);
    EXPECT_DOUBLE_EQ(rot.y, 0.0);
    EXPECT_DOUBLE_EQ(rot.z, 0.0);
}

TEST(RotationTest, QuaternionNormalization) {
    Rotation rot;
    rot.w = 0.5;
    rot.x = 0.5;
    rot.y = 0.5;
    rot.z = 0.5;
    
    double magnitude = std::sqrt(rot.w * rot.w + rot.x * rot.x + 
                                 rot.y * rot.y + rot.z * rot.z);
    
    EXPECT_DOUBLE_EQ(magnitude, 1.0);
}

TEST(PoseTest, DefaultConstruction) {
    Pose pose;
    
    EXPECT_EQ(pose.position.x, 0.0);
    EXPECT_EQ(pose.position.y, 0.0);
    EXPECT_EQ(pose.position.z, 0.0);
    
    EXPECT_EQ(pose.rotation.w, 0.0);
    EXPECT_EQ(pose.rotation.x, 0.0);
    EXPECT_EQ(pose.rotation.y, 0.0);
    EXPECT_EQ(pose.rotation.z, 0.0);
}

TEST(PoseTest, ValueAssignment) {
    Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    
    pose.rotation.w = 1.0;
    pose.rotation.x = 0.0;
    pose.rotation.y = 0.0;
    pose.rotation.z = 0.0;
    
    EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
    
    EXPECT_DOUBLE_EQ(pose.rotation.w, 1.0);
    EXPECT_DOUBLE_EQ(pose.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.z, 0.0);
}

}  // namespace auto_battlebot
