#include <gtest/gtest.h>

#include "remote_command.hpp"

namespace auto_battlebot {

TEST(RemoteCommandTest, ParsesLowercaseCommandTopic) {
    EXPECT_EQ(parse_remote_command_topic("/command/reinit_field"), RemoteCommand::REINIT_FIELD);
    EXPECT_EQ(parse_remote_command_topic("/command/REINIT_FIELD"), RemoteCommand::REINIT_FIELD);
}

TEST(RemoteCommandTest, RejectsUnknownAndForeignTopics) {
    EXPECT_EQ(parse_remote_command_topic("/command/not_a_command"), std::nullopt);
    EXPECT_EQ(parse_remote_command_topic("/command/"), std::nullopt);
    EXPECT_EQ(parse_remote_command_topic("/reinit_field"), std::nullopt);
}

}  // namespace auto_battlebot
