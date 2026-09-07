#include <gtest/gtest.h>

#include "viz/frame.hpp"

namespace auto_battlebot {

TEST(VizFrameTest, AdvertiseRoundTrip) {
    viz::AdvertiseFrame in;
    in.channel_id = 7;
    in.latch = true;
    in.topic = "/field_points";
    in.message_encoding = "protobuf";
    in.schema_name = "foxglove.PointCloud";
    in.schema_encoding = "protobuf";
    in.schema_data = std::string("\x00\x01\x02binary", 9);

    auto bytes = viz::encode_advertise(in);
    ASSERT_GT(bytes.size(), viz::kFrameHeaderBytes);
    uint32_t len = 0;
    size_t off = 0;
    ASSERT_TRUE(viz::detail::get_u32(bytes.data(), bytes.size(), off, len));
    EXPECT_EQ(len, bytes.size() - viz::kFrameHeaderBytes);

    const std::byte* body = bytes.data() + viz::kFrameHeaderBytes;
    EXPECT_EQ(viz::frame_kind(body, len), viz::FrameKind::ADVERTISE);
    viz::AdvertiseFrame out;
    ASSERT_TRUE(viz::decode_advertise(body, len, out));
    EXPECT_EQ(out.channel_id, 7u);
    EXPECT_TRUE(out.latch);
    EXPECT_EQ(out.topic, "/field_points");
    EXPECT_EQ(out.message_encoding, "protobuf");
    EXPECT_EQ(out.schema_name, "foxglove.PointCloud");
    EXPECT_EQ(out.schema_encoding, "protobuf");
    EXPECT_EQ(out.schema_data, in.schema_data);
}

TEST(VizFrameTest, MessageRoundTrip) {
    const std::string payload = "hello";
    auto bytes =
        viz::encode_message(3, 1234567890123456789ull,
                            reinterpret_cast<const std::byte*>(payload.data()), payload.size());
    const std::byte* body = bytes.data() + viz::kFrameHeaderBytes;
    const size_t len = bytes.size() - viz::kFrameHeaderBytes;
    EXPECT_EQ(viz::frame_kind(body, len), viz::FrameKind::MESSAGE);
    viz::MessageFrame out;
    ASSERT_TRUE(viz::decode_message(body, len, out));
    EXPECT_EQ(out.channel_id, 3u);
    EXPECT_EQ(out.log_time_ns, 1234567890123456789ull);
    ASSERT_EQ(out.payload_len, payload.size());
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(out.payload), out.payload_len), payload);
}

TEST(VizFrameTest, SubscriberCountRoundTrip) {
    auto bytes = viz::encode_subscriber_count(9, 2);
    const std::byte* body = bytes.data() + viz::kFrameHeaderBytes;
    const size_t len = bytes.size() - viz::kFrameHeaderBytes;
    EXPECT_EQ(viz::frame_kind(body, len), viz::FrameKind::SUBSCRIBER_COUNT);
    viz::SubscriberCountFrame out;
    ASSERT_TRUE(viz::decode_subscriber_count(body, len, out));
    EXPECT_EQ(out.channel_id, 9u);
    EXPECT_EQ(out.count, 2u);
}

TEST(VizFrameTest, TruncatedFramesAreRejected) {
    viz::AdvertiseFrame in;
    in.topic = "/x";
    auto bytes = viz::encode_advertise(in);
    const std::byte* body = bytes.data() + viz::kFrameHeaderBytes;
    viz::AdvertiseFrame out;
    EXPECT_FALSE(viz::decode_advertise(body, 6, out));
    viz::MessageFrame msg;
    EXPECT_FALSE(viz::decode_message(body, 4, msg));
    EXPECT_FALSE(viz::frame_kind(body, 0).has_value());
    const std::byte bad[1] = {std::byte{99}};
    EXPECT_FALSE(viz::frame_kind(bad, 1).has_value());
}

}  // namespace auto_battlebot
