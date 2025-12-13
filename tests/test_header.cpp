#include <gtest/gtest.h>
#include "data_structures/header.hpp"

namespace auto_battlebot {

TEST(HeaderTest, DefaultConstruction) {
    Header header;
    // Test that we can construct a Header
    EXPECT_EQ(header.timestamp, 0.0);
    EXPECT_EQ(header.frame_id, "");
}

TEST(HeaderTest, ValueAssignment) {
    Header header;
    header.timestamp = 1234.5678;
    header.frame_id = "base_link";
    
    EXPECT_DOUBLE_EQ(header.timestamp, 1234.5678);
    EXPECT_EQ(header.frame_id, "base_link");
}

TEST(HeaderTest, CopyConstruction) {
    Header header1;
    header1.timestamp = 9876.5432;
    header1.frame_id = "world";
    
    Header header2 = header1;
    
    EXPECT_DOUBLE_EQ(header2.timestamp, 9876.5432);
    EXPECT_EQ(header2.frame_id, "world");
}

}  // namespace auto_battlebot
