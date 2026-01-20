#include <gtest/gtest.h>
#include "data_structures/keypoint.hpp"

namespace auto_battlebot
{
    TEST(KeypointTest, DefaultConstruction)
    {
        Keypoint kp;

        EXPECT_EQ(kp.label, Label::EMPTY);
        EXPECT_EQ(kp.keypoint_label, KeypointLabel::EMPTY);
        EXPECT_FLOAT_EQ(kp.x, 0.0f);
        EXPECT_FLOAT_EQ(kp.y, 0.0f);
    }

    TEST(KeypointTest, ValueAssignment)
    {
        Keypoint kp;
        kp.label = Label::MR_STABS_MK1;
        kp.keypoint_label = KeypointLabel::MR_STABS_MK1_FRONT;
        kp.x = 320.5f;
        kp.y = 240.5f;

        EXPECT_EQ(kp.label, Label::MR_STABS_MK1);
        EXPECT_EQ(kp.keypoint_label, KeypointLabel::MR_STABS_MK1_FRONT);
        EXPECT_FLOAT_EQ(kp.x, 320.5f);
        EXPECT_FLOAT_EQ(kp.y, 240.5f);
    }

    TEST(KeypointsStampedTest, DefaultConstruction)
    {
        KeypointsStamped stamped;

        EXPECT_EQ(stamped.header.stamp, 0.0);
        EXPECT_EQ(stamped.header.frame_id, FrameId::EMPTY);
        EXPECT_TRUE(stamped.keypoints.empty());
    }

    TEST(KeypointsStampedTest, AddKeypoint)
    {
        KeypointsStamped stamped;
        stamped.header.stamp = 123.456;
        stamped.header.frame_id = FrameId::CAMERA;

        Keypoint kp;
        kp.label = Label::MR_STABS_MK1;
        kp.keypoint_label = KeypointLabel::MR_STABS_MK1_FRONT;
        kp.x = 320.0f;
        kp.y = 240.0f;

        stamped.keypoints.push_back(kp);

        EXPECT_EQ(stamped.keypoints.size(), 1);
        EXPECT_EQ(stamped.keypoints[0].label, Label::MR_STABS_MK1);
        EXPECT_EQ(stamped.keypoints[0].keypoint_label, KeypointLabel::MR_STABS_MK1_FRONT);
        EXPECT_FLOAT_EQ(stamped.keypoints[0].x, 320.0f);
        EXPECT_FLOAT_EQ(stamped.keypoints[0].y, 240.0f);
    }

    TEST(KeypointsStampedTest, MultipleKeypoints)
    {
        KeypointsStamped stamped;

        Keypoint kp1;
        kp1.label = Label::MR_STABS_MK1;
        kp1.keypoint_label = KeypointLabel::MR_STABS_MK1_FRONT;
        kp1.x = 320.0f;
        kp1.y = 240.0f;

        Keypoint kp2;
        kp2.label = Label::MR_STABS_MK1;
        kp2.keypoint_label = KeypointLabel::MR_STABS_MK1_BACK;
        kp2.x = 330.0f;
        kp2.y = 240.0f;

        stamped.keypoints.push_back(kp1);
        stamped.keypoints.push_back(kp2);

        EXPECT_EQ(stamped.keypoints.size(), 2);
        EXPECT_EQ(stamped.keypoints[0].keypoint_label, KeypointLabel::MR_STABS_MK1_FRONT);
        EXPECT_EQ(stamped.keypoints[1].keypoint_label, KeypointLabel::MR_STABS_MK1_BACK);
    }
} // namespace auto_battlebot
