#include <gtest/gtest.h>

#include <set>
#include <string>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {
namespace {

struct Inner {
    int n = 0;
    AB_JSON_MESSAGE(Inner, "test.Inner", n)
};

struct EveryKind {
    bool flag = false;
    int count = 0;
    double value = 0.0;
    std::string name;
    SystemAction action = SystemAction::REBOOT_HOST;
    std::vector<int> list;
    std::optional<double> maybe;
    Inner inner;
    AB_JSON_MESSAGE(EveryKind, "test.EveryKind", flag, count, value, name, action, list, maybe,
                    inner)
};

TEST(RemoteProtocolTest, TopicsAreUniqueAndPrefixed) {
    std::set<std::string> seen;
    for_each_status_topic([&](const auto& row) {
        EXPECT_TRUE(row.topic.starts_with("/status/")) << row.topic;
        EXPECT_TRUE(seen.insert(std::string(row.topic)).second) << row.topic;
    });
    for_each_command_topic([&](const auto& row) {
        EXPECT_TRUE(row.topic.starts_with("/command/")) << row.topic;
        EXPECT_TRUE(seen.insert(std::string(row.topic)).second) << row.topic;
    });
    EXPECT_EQ(seen.size(), std::tuple_size_v<StatusRows> + std::tuple_size_v<CommandRows>);
}

TEST(RemoteProtocolTest, EveryCommandRoundTrips) {
    for_each_command_topic([&](const auto& row) {
        using Payload = typename std::remove_cvref_t<decltype(row)>::payload_type;
        Payload original{};
        nlohmann::json j;
        to_json(j, original);
        Payload parsed{};
        from_json(j, parsed);
        nlohmann::json again;
        to_json(again, parsed);
        EXPECT_EQ(j, again) << row.topic;
    });
}

TEST(RemoteProtocolTest, SchemaCoversEveryFieldKind) {
    const std::string expected =
        R"({"properties":{"action":{"enum":["reboot_host","poweroff_host"],"type":"string"},)"
        R"("count":{"type":"integer"},"flag":{"type":"boolean"},)"
        R"("inner":{"properties":{"n":{"type":"integer"}},"required":["n"],"title":"test.Inner",)"
        R"("type":"object"},"list":{"items":{"type":"integer"},"type":"array"},)"
        R"("maybe":{"type":"number"},"name":{"type":"string"},"value":{"type":"number"}},)"
        R"("required":["flag","count","value","name","action","list","inner"],)"
        R"("title":"test.EveryKind","type":"object"})";
    EXPECT_EQ(schema_for<EveryKind>(), expected);
}

TEST(RemoteProtocolTest, EnumsAndOptionalsEncodeAsDocumented) {
    EveryKind message;
    message.action = SystemAction::POWEROFF_HOST;
    nlohmann::json j;
    to_json(j, message);
    EXPECT_EQ(j["action"], "poweroff_host");
    EXPECT_FALSE(j.contains("maybe"));
    message.maybe = 1.5;
    to_json(j, message);
    EXPECT_DOUBLE_EQ(j["maybe"].get<double>(), 1.5);
}

TEST(RemoteProtocolTest, CommandTopicOfNamesTheVariantsRow) {
    EXPECT_EQ(command_topic_of(RemoteCommand{SetAutonomyCommand{}}), "/command/set_autonomy");
    EXPECT_EQ(command_topic_of(RemoteCommand{ReinitFieldCommand{}}), "/command/reinit_field");
}

}  // namespace
}  // namespace auto_battlebot::remote
