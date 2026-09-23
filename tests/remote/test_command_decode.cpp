#include <gtest/gtest.h>

#include <string>

#include "remote/command_decode.hpp"

namespace auto_battlebot::remote {
namespace {

std::optional<RemoteCommand> decode(std::string_view topic, const std::string& text) {
    return decode_command(topic, reinterpret_cast<const std::byte*>(text.data()), text.size());
}

TEST(CommandDecodeTest, ValidPayloadDecodes) {
    auto command = decode("/command/set_opponent_count", R"({"count": 2})");
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(std::get<SetOpponentCountCommand>(*command).count, 2);

    command = decode("/command/system_action", R"({"action": "poweroff_host"})");
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(std::get<SystemActionCommand>(*command).action, SystemAction::POWEROFF_HOST);
}

TEST(CommandDecodeTest, EmptyObjectAndEmptyPayloadReinitField) {
    EXPECT_TRUE(decode("/command/reinit_field", "{}").has_value());
    EXPECT_TRUE(decode("/command/reinit_field", "").has_value());
    // Foxglove publish panels sometimes carry extra keys; an empty message ignores them.
    EXPECT_TRUE(decode("/command/reinit_field", R"({"data": 1})").has_value());
}

TEST(CommandDecodeTest, BadInputReturnsNulloptWithoutThrowing) {
    EXPECT_FALSE(decode("/command/set_opponent_count", "{}").has_value());
    EXPECT_FALSE(decode("/command/set_opponent_count", R"({"count": "2"})").has_value());
    EXPECT_FALSE(decode("/command/set_opponent_count", R"({"count": 2.5})").has_value());
    EXPECT_FALSE(decode("/command/set_autonomy", R"({"enabled": 1})").has_value());
    EXPECT_FALSE(decode("/command/system_action", R"({"action": "explode"})").has_value());
    EXPECT_FALSE(decode("/command/set_autonomy", "{not json").has_value());
    EXPECT_FALSE(decode("/command/set_autonomy", "[]").has_value());
    EXPECT_FALSE(decode("/command/not_a_command", "{}").has_value());
    EXPECT_FALSE(decode("/reinit_field", "{}").has_value());
}

}  // namespace
}  // namespace auto_battlebot::remote
