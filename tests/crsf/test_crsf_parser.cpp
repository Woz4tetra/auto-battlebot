#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "crsf/crsf_parser.hpp"

namespace auto_battlebot {
namespace {

struct CapturedFrame {
    std::string name;
    std::vector<uint8_t> bytes;
    std::map<std::string, double> decoded;
};

std::filesystem::path capture_path() {
    return std::filesystem::path(__FILE__).parent_path().parent_path() / "fixtures" / "crsf" /
           "edgetx_telemetry_capture.log";
}

bool is_hex_byte(const std::string &token) {
    return token.size() == 2 && std::isxdigit(static_cast<unsigned char>(token[0])) &&
           std::isxdigit(static_cast<unsigned char>(token[1]));
}

std::vector<CapturedFrame> load_capture() {
    std::ifstream file(capture_path());
    std::vector<CapturedFrame> frames;
    std::string line;
    while (std::getline(file, line)) {
        if (line.rfind("addr=", 0) != 0) continue;
        std::istringstream tokens(line);
        std::string token;
        CapturedFrame frame;
        tokens >> token >> frame.name;
        while (tokens >> token) {
            const auto eq = token.find('=');
            if (eq != std::string::npos) {
                frame.decoded[token.substr(0, eq)] = std::stod(token.substr(eq + 1));
            } else if (is_hex_byte(token) && frame.decoded.empty()) {
                frame.bytes.push_back(static_cast<uint8_t>(std::stoul(token, nullptr, 16)));
            }
        }
        frames.push_back(std::move(frame));
    }
    return frames;
}

template <typename T>
T only_packet(const std::vector<CrsfParseResult> &results) {
    EXPECT_EQ(results.size(), 1u);
    EXPECT_FALSE(results.front().second.has_value());
    EXPECT_TRUE(results.front().first.has_value());
    return std::get<T>(*results.front().first);
}

TEST(CrsfParserCaptureTest, CaptureHoldsBothFrameTypes) {
    const auto frames = load_capture();
    int attitude = 0;
    int battery = 0;
    for (const auto &frame : frames) {
        attitude += frame.name == "attitude" ? 1 : 0;
        battery += frame.name == "battery" ? 1 : 0;
    }
    EXPECT_GT(attitude, 50);
    EXPECT_GT(battery, 50);
}

TEST(CrsfParserCaptureTest, EveryCapturedEdgeTxFrameDecodesToWhatTheCaptureScriptRead) {
    for (const auto &frame : load_capture()) {
        SCOPED_TRACE(frame.name);
        CrsfParser parser;
        const auto results = parser.parse(frame.bytes);
        EXPECT_EQ(parser.crc_valid_frames(), 1u);
        if (frame.name == "attitude") {
            const auto pkt = only_packet<CrsfAttitude>(results);
            EXPECT_NEAR(pkt.pitch_rad, frame.decoded.at("Ptch"), 1e-4);
            EXPECT_NEAR(pkt.roll_rad, frame.decoded.at("Roll"), 1e-4);
            EXPECT_NEAR(pkt.yaw_rad, frame.decoded.at("Yaw"), 1e-4);
        } else if (frame.name == "battery") {
            const auto pkt = only_packet<CrsfBattery>(results);
            EXPECT_NEAR(pkt.voltage_v, frame.decoded.at("RxBt"), 1e-4);
            EXPECT_NEAR(pkt.current_a, frame.decoded.at("Curr"), 1e-4);
            EXPECT_NEAR(pkt.consumption_mah, frame.decoded.at("Capa"), 1e-4);
            EXPECT_EQ(pkt.remaining_pct, static_cast<uint8_t>(frame.decoded.at("Bat%")));
        }
    }
}

TEST(CrsfParserCaptureTest, CapturedFramesReencodeByteForByte) {
    for (const auto &frame : load_capture()) {
        SCOPED_TRACE(frame.name);
        CrsfParser parser;
        const auto results = parser.parse(frame.bytes);
        ASSERT_EQ(results.size(), 1u);
        ASSERT_TRUE(results.front().first);
        EXPECT_EQ(encode_crsf_packet(*results.front().first, frame.bytes.front()), frame.bytes);
    }
}

TEST(CrsfParserCaptureTest, WholeCaptureAsOneStreamSplitAtOddBoundaries) {
    std::vector<uint8_t> stream;
    size_t frames = 0;
    for (const auto &frame : load_capture()) {
        stream.insert(stream.end(), frame.bytes.begin(), frame.bytes.end());
        ++frames;
    }
    CrsfParser parser;
    size_t parsed = 0;
    for (size_t pos = 0; pos < stream.size(); pos += 7) {
        const size_t n = std::min<size_t>(7, stream.size() - pos);
        for (const auto &[packet, error] : parser.parse(stream.data() + pos, n)) {
            EXPECT_FALSE(error.has_value());
            parsed += packet ? 1 : 0;
        }
    }
    EXPECT_EQ(parsed, frames);
    EXPECT_EQ(parser.crc_valid_frames(), frames);
}

TEST(CrsfParserRoundTripTest, LinkStatisticsRoundTrips) {
    CrsfLinkStatistics stats;
    stats.up_rssi_ant1_neg_dbm = 61;
    stats.up_rssi_ant2_neg_dbm = 70;
    stats.up_link_quality_pct = 99;
    stats.up_snr_db = -4;
    stats.active_antenna = 1;
    stats.rf_profile = CrsfRfProfile::FPS_150;
    stats.up_rf_power = CrsfRfPower::MW_250;
    stats.down_rssi_neg_dbm = 55;
    stats.down_link_quality_pct = 100;
    stats.down_snr_db = 9;
    CrsfParser parser;
    const auto pkt = only_packet<CrsfLinkStatistics>(parser.parse(encode_crsf_packet(stats)));
    EXPECT_EQ(pkt.up_rssi_ant1_neg_dbm, 61);
    EXPECT_EQ(pkt.up_rssi_ant2_neg_dbm, 70);
    EXPECT_EQ(pkt.up_link_quality_pct, 99);
    EXPECT_EQ(pkt.up_snr_db, -4);
    EXPECT_EQ(pkt.active_antenna, 1);
    EXPECT_EQ(pkt.rf_profile, CrsfRfProfile::FPS_150);
    EXPECT_EQ(pkt.up_rf_power, CrsfRfPower::MW_250);
    EXPECT_EQ(pkt.down_rssi_neg_dbm, 55);
    EXPECT_EQ(pkt.down_link_quality_pct, 100);
    EXPECT_EQ(pkt.down_snr_db, 9);
}

TEST(CrsfParserRoundTripTest, NegativeAttitudeAndFlightModeRoundTrip) {
    CrsfParser parser;
    CrsfAttitude attitude;
    attitude.pitch_rad = -1.2345f;
    attitude.roll_rad = 0.5f;
    attitude.yaw_rad = -3.1f;
    const auto a = only_packet<CrsfAttitude>(parser.parse(encode_crsf_packet(attitude)));
    EXPECT_NEAR(a.pitch_rad, -1.2345f, 1e-4);
    EXPECT_NEAR(a.roll_rad, 0.5f, 1e-4);
    EXPECT_NEAR(a.yaw_rad, -3.1f, 1e-4);

    CrsfFlightMode mode;
    mode.flight_mode = "ACRO";
    EXPECT_EQ(only_packet<CrsfFlightMode>(parser.parse(encode_crsf_packet(mode))).flight_mode,
              "ACRO");
}

TEST(CrsfParserErrorTest, CorruptedCrcIsReportedAndNotCounted) {
    CrsfBattery battery;
    battery.voltage_v = 16.5f;
    std::vector<uint8_t> frame = encode_crsf_packet(battery);
    frame.back() ^= 0xFF;
    CrsfParser parser;
    const auto results = parser.parse(frame);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results.front().first);
    ASSERT_TRUE(results.front().second);
    EXPECT_NE(results.front().second->message.find("CRC"), std::string::npos);
    EXPECT_EQ(parser.crc_valid_frames(), 0u);
}

TEST(CrsfParserErrorTest, UnsupportedTypeStillCountsAsTelemetryArriving) {
    std::vector<uint8_t> frame = {kCrsfRadioAddress, 0x04, 0x3A, 0x01, 0x02};
    frame.push_back(crsf_crc8(frame.data() + 2, frame.size() - 2));
    CrsfParser parser;
    const auto results = parser.parse(frame);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_FALSE(results.front().first);
    EXPECT_EQ(parser.crc_valid_frames(), 1u);
}

TEST(CrsfParserErrorTest, GarbageBeforeAFrameIsSkipped) {
    CrsfBattery battery;
    battery.voltage_v = 12.3f;
    std::vector<uint8_t> stream = {0x11, 0x22, 0x33};
    const auto frame = encode_crsf_packet(battery);
    stream.insert(stream.end(), frame.begin(), frame.end());
    CrsfParser parser;
    EXPECT_NEAR(only_packet<CrsfBattery>(parser.parse(stream)).voltage_v, 12.3f, 1e-4);
}

}  // namespace
}  // namespace auto_battlebot

namespace auto_battlebot {
namespace {

std::vector<uint8_t> opentx_channel_packet(uint8_t phase) {
    std::vector<uint8_t> packet = {0xA3, 0xA4, 0xA5, phase, 17};
    for (int i = 0; i < 8; ++i) {
        packet.push_back(i == 5 ? 0xE8 : 0x00);
        packet.push_back(i == 5 ? 0x03 : 0x00);
    }
    packet.push_back(0x00);
    return packet;
}

TEST(CrsfParserStreamTest, FramesInterleavedWithTheChannelStreamAllArrive) {
    CrsfLinkStatistics stats;
    stats.up_link_quality_pct = 100;
    const auto crsf = encode_crsf_packet(stats);
    std::vector<uint8_t> stream;
    for (int i = 0; i < 20; ++i) {
        for (uint8_t phase : {uint8_t{0}, uint8_t{1}}) {
            const auto packet = opentx_channel_packet(phase);
            stream.insert(stream.end(), packet.begin(), packet.end());
        }
        stream.insert(stream.end(), crsf.begin(), crsf.end());
    }
    CrsfParser parser;
    int link_statistics = 0;
    for (const auto &[packet, error] : parser.parse(stream)) {
        if (packet && std::holds_alternative<CrsfLinkStatistics>(*packet)) ++link_statistics;
    }
    EXPECT_EQ(link_statistics, 20);
}

}  // namespace
}  // namespace auto_battlebot
