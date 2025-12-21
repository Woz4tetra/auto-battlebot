#include <gtest/gtest.h>
#include "data_structures/header.hpp"

namespace auto_battlebot
{
    TEST(HeaderTest, DefaultConstruction)
    {
        Header header;
        // Test that we can construct a Header
        EXPECT_EQ(header.stamp, 0.0);
        EXPECT_EQ(header.frame_id, FrameId::EMPTY);
    }

    TEST(HeaderTest, ValueAssignment)
    {
        Header header;
        header.stamp = 1234.5678;
        header.frame_id = FrameId::OUR_ROBOT_1;

        EXPECT_DOUBLE_EQ(header.stamp, 1234.5678);
        EXPECT_EQ(header.frame_id, FrameId::OUR_ROBOT_1);
    }

    TEST(HeaderTest, CopyConstruction)
    {
        Header header1;
        header1.stamp = 9876.5432;
        header1.frame_id = FrameId::FIELD;

        Header header2 = header1;

        EXPECT_DOUBLE_EQ(header2.stamp, 9876.5432);
        EXPECT_EQ(header2.frame_id, FrameId::FIELD);
    }
} // namespace auto_battlebot
