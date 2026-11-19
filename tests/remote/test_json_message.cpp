#include <gtest/gtest.h>

#include <stdexcept>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {
namespace {

template <typename T>
void parse(const char *text, T &out) {
    from_json(nlohmann::json::parse(text), out);
}

TEST(JsonMessageTest, DecodeRefusesCoercion) {
    SetOpponentCountCommand count;
    EXPECT_THROW(parse(R"({"count": "3"})", count), std::invalid_argument);
    EXPECT_THROW(parse(R"({"count": 1.5})", count), std::invalid_argument);
    SetAutonomyCommand autonomy;
    EXPECT_THROW(parse(R"({"enabled": 1})", autonomy), std::invalid_argument);
    parse(R"({"count": 3})", count);
    EXPECT_EQ(count.count, 3);
}

TEST(JsonMessageTest, MissingRequiredFieldThrows) {
    SetOpponentCountCommand count;
    EXPECT_THROW(parse("{}", count), std::invalid_argument);
    EXPECT_EQ(count.count, 1);
}

TEST(JsonMessageTest, MissingOptionalFieldStaysEmpty) {
    SetManualTargetCommand target;
    parse("{}", target);
    EXPECT_FALSE(target.field.has_value());
    EXPECT_FALSE(target.image.has_value());
}

}  // namespace
}  // namespace auto_battlebot::remote
