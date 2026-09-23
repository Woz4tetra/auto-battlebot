#include <gtest/gtest.h>

#include "remote/status_bus.hpp"

namespace auto_battlebot::remote {
namespace {

TEST(StatusBusTest, ThrottlesInsideTheRateWindow) {
    // No sink and no recorder: the channels exist and publish is a no-op past the throttle.
    StatusBus bus(nullptr, nullptr);
    SystemStatusMessage message;
    EXPECT_TRUE(bus.publish(message));
    // /status/system is 10 Hz, so a second call within 100 ms is dropped.
    EXPECT_FALSE(bus.publish(message));
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    EXPECT_TRUE(bus.publish(message));
}

TEST(StatusBusTest, UnthrottledTopicPublishesEveryCall) {
    StatusBus bus(nullptr, nullptr);
    CommandAckMessage ack;
    EXPECT_TRUE(bus.publish(ack));
    EXPECT_TRUE(bus.publish(ack));
}

TEST(StatusBusTest, PayloadParsesBackToTheStruct) {
    SystemStatusMessage message;
    message.loop_rate_hz = 57.5;
    message.jetson_temperature_c = 48.0;
    message.compute_mode = "MAXN";
    const std::string text = to_json_string(message);
    SystemStatusMessage parsed;
    from_json(nlohmann::json::parse(text), parsed);
    EXPECT_DOUBLE_EQ(parsed.loop_rate_hz, 57.5);
    ASSERT_TRUE(parsed.jetson_temperature_c.has_value());
    EXPECT_DOUBLE_EQ(*parsed.jetson_temperature_c, 48.0);
    EXPECT_EQ(parsed.compute_mode, "MAXN");
}

TEST(StatusBusTest, LatchedTopicsAdvertiseLatched) {
    EXPECT_EQ(status_topic<AppInfoMessage>().latch, Latch::YES);
    EXPECT_EQ(status_topic<NetworkMessage>().latch, Latch::YES);
    EXPECT_EQ(status_topic<SystemStatusMessage>().latch, Latch::NO);
}

}  // namespace
}  // namespace auto_battlebot::remote
