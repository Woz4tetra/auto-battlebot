#include <gtest/gtest.h>
#include "data_structures/robot.hpp"

namespace auto_battlebot
{
    TEST(RobotConfigTest, DefaultConstruction)
    {
        RobotConfig config;

        // Verify default values are set
        EXPECT_EQ(config.label, Label::MR_STABS_MK1);
        EXPECT_EQ(config.group, Group::OURS);
    }

    TEST(RobotConfigTest, ValueAssignment)
    {
        RobotConfig config;
        config.label = Label::MR_STABS_MK2;
        config.group = Group::OURS;

        EXPECT_EQ(config.label, Label::MR_STABS_MK2);
        EXPECT_EQ(config.group, Group::OURS);
    }

    TEST(RobotDescriptionTest, DefaultConstruction)
    {
        RobotDescription desc;

        // Verify default values are set
        EXPECT_EQ(desc.label, Label::MR_STABS_MK1);
        EXPECT_EQ(desc.group, Group::OURS);
        EXPECT_EQ(desc.pose.position.x, 0.0);
        EXPECT_EQ(desc.pose.position.y, 0.0);
        EXPECT_EQ(desc.pose.position.z, 0.0);
        EXPECT_EQ(desc.size.x, 0.0);
        EXPECT_EQ(desc.size.y, 0.0);
        EXPECT_EQ(desc.size.z, 0.0);
    }

    TEST(RobotDescriptionTest, ValueAssignment)
    {
        RobotDescription desc;
        desc.label = Label::MR_STABS_MK2;
        desc.group = Group::OURS;
        desc.pose.position.x = 1.0;
        desc.pose.position.y = 2.0;
        desc.pose.position.z = 0.0;
        desc.pose.rotation.w = 1.0;
        desc.size.x = 0.3;
        desc.size.y = 0.3;
        desc.size.z = 0.2;

        EXPECT_EQ(desc.label, Label::MR_STABS_MK2);
        EXPECT_EQ(desc.group, Group::OURS);
        EXPECT_DOUBLE_EQ(desc.pose.position.x, 1.0);
        EXPECT_DOUBLE_EQ(desc.pose.position.y, 2.0);
        EXPECT_DOUBLE_EQ(desc.size.x, 0.3);
        EXPECT_DOUBLE_EQ(desc.size.y, 0.3);
        EXPECT_DOUBLE_EQ(desc.size.z, 0.2);
    }

    TEST(RobotDescriptionsStampedTest, DefaultConstruction)
    {
        RobotDescriptionsStamped stamped;

        EXPECT_EQ(stamped.header.timestamp, 0.0);
        EXPECT_EQ(stamped.header.frame_id, "");
        EXPECT_TRUE(stamped.descriptions.empty());
    }

    TEST(RobotDescriptionsStampedTest, AddDescription)
    {
        RobotDescriptionsStamped stamped;
        stamped.header.timestamp = 123.456;
        stamped.header.frame_id = "world";

        RobotDescription desc;
        desc.label = Label::MR_STABS_MK2;
        desc.group = Group::OURS;

        stamped.descriptions.push_back(desc);

        EXPECT_EQ(stamped.descriptions.size(), 1);
        EXPECT_EQ(stamped.descriptions[0].label, Label::MR_STABS_MK2);
        EXPECT_EQ(stamped.descriptions[0].group, Group::OURS);
    }

    TEST(RobotDescriptionsStampedTest, MultipleDescriptions)
    {
        RobotDescriptionsStamped stamped;

        RobotDescription desc1;
        desc1.label = Label::MR_STABS_MK2;
        desc1.group = Group::OURS;

        RobotDescription desc2;
        desc2.label = Label::OPPONENT;
        desc2.group = Group::THEIRS;

        stamped.descriptions.push_back(desc1);
        stamped.descriptions.push_back(desc2);

        EXPECT_EQ(stamped.descriptions.size(), 2);
        EXPECT_EQ(stamped.descriptions[0].label, Label::MR_STABS_MK2);
        EXPECT_EQ(stamped.descriptions[1].label, Label::OPPONENT);
    }
} // namespace auto_battlebot
