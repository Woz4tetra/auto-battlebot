#include <fcntl.h>
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "channels/channels_parser.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "time/manual_clock.hpp"
#include "transmitter/opentx_transmitter.hpp"

namespace auto_battlebot {
namespace {

constexpr int kInitButtonChannel = 7;
constexpr int kTrainerChannel = 5;
constexpr int kPressed = 1000;
constexpr int kReleased = -1000;
constexpr double kMaxMotorRpm = 1000.0;
constexpr double kMaxMotorRpmPerS = 500.0;
constexpr double kStepS = 0.1;

std::vector<uint8_t> channel_frame(const std::array<int16_t, kMaxChannels> &channels) {
    std::vector<uint8_t> bytes;
    for (uint8_t phase = 0; phase < 2; ++phase) {
        const uint8_t length = kChannelsPerPacket * 2 + 1;
        std::vector<uint8_t> packet = {0xA3, 0xA4, 0xA5, phase, length};
        uint8_t checksum = phase ^ length;
        for (int i = 0; i < kChannelsPerPacket; ++i) {
            const int16_t value = channels[phase * kChannelsPerPacket + i];
            const uint8_t low = static_cast<uint8_t>(value & 0xFF);
            const uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
            packet.push_back(low);
            packet.push_back(high);
            checksum ^= low;
            checksum ^= high;
        }
        packet.push_back(checksum);
        bytes.insert(bytes.end(), packet.begin(), packet.end());
    }
    return bytes;
}

class OpenTxTransmitterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        char name[256];
        ASSERT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
        path_ = name;
        fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK);
        DiagnosticsLogger::remove_logger("opentx_transmitter");
        clock_ = std::make_shared<ManualClock>();
    }

    void TearDown() override {
        transmitter_.reset();
        if (master_ >= 0) close(master_);
        if (slave_ >= 0) close(slave_);
        DiagnosticsLogger::remove_logger("opentx_transmitter");
    }

    void build(OpenTxTransmitterConfiguration config) {
        transmitter_ = std::make_unique<OpenTxTransmitter>(config, clock_,
                                                           [this]() -> std::optional<std::string> {
                                                               if (!device_present_)
                                                                   return std::nullopt;
                                                               return path_;
                                                           });
    }

    static OpenTxTransmitterConfiguration button_config() {
        OpenTxTransmitterConfiguration config;
        config.init_button_channel = kInitButtonChannel;
        config.trainer_enable_channel = kTrainerChannel;
        return config;
    }

    static OpenTxTransmitterConfiguration limited_config() {
        OpenTxTransmitterConfiguration config;
        config.trainer_enable_channel = kTrainerChannel;
        config.max_motor_rpm = kMaxMotorRpm;
        config.max_motor_rpm_per_s = kMaxMotorRpmPerS;
        return config;
    }

    void feed(int init_button, int trainer) {
        std::array<int16_t, kMaxChannels> channels{};
        channels[kInitButtonChannel] = static_cast<int16_t>(init_button);
        channels[kTrainerChannel] = static_cast<int16_t>(trainer);
        feed_channels(channels);
    }

    void feed_channels(const std::array<int16_t, kMaxChannels> &channels) {
        const std::vector<uint8_t> bytes = channel_frame(channels);
        ASSERT_EQ(write(master_, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
        usleep(2000);
        transmitter_->update();
    }

    std::string read_master() {
        std::string text;
        char buffer[512];
        for (int i = 0; i < 20; ++i) {
            const ssize_t n = read(master_, buffer, sizeof(buffer));
            if (n > 0) {
                text.append(buffer, static_cast<size_t>(n));
            } else if (!text.empty()) {
                break;
            } else {
                usleep(1000);
            }
        }
        return text;
    }

    std::optional<int> last_linear_trainer_value(const std::string &text) {
        std::optional<int> value;
        const std::regex pattern("trainer 0 (-?[0-9]+)");
        for (auto it = std::sregex_iterator(text.begin(), text.end(), pattern);
             it != std::sregex_iterator(); ++it) {
            value = std::stoi((*it)[1].str());
        }
        return value;
    }

    int master_ = -1;
    int slave_ = -1;
    std::string path_;
    bool device_present_ = true;
    std::shared_ptr<ManualClock> clock_;
    std::unique_ptr<OpenTxTransmitter> transmitter_;
};

TEST_F(OpenTxTransmitterTest, InitButtonFiresOncePerRisingEdge) {
    build(button_config());
    ASSERT_TRUE(transmitter_->initialize());

    feed(kReleased, 0);
    EXPECT_FALSE(transmitter_->did_init_button_press());
    feed(kPressed, 0);
    EXPECT_TRUE(transmitter_->did_init_button_press());
    EXPECT_FALSE(transmitter_->did_init_button_press()) << "fired again on the same press";
    feed(kPressed, 0);
    EXPECT_FALSE(transmitter_->did_init_button_press()) << "fired while the button stayed held";
    feed(kReleased, 0);
    EXPECT_FALSE(transmitter_->did_init_button_press());
    feed(kPressed, 0);
    EXPECT_TRUE(transmitter_->did_init_button_press()) << "second press after release was lost";
}

TEST_F(OpenTxTransmitterTest, UnsetInitButtonChannelNeverFires) {
    OpenTxTransmitterConfiguration config;
    config.trainer_enable_channel = kTrainerChannel;
    ASSERT_FALSE(config.init_button_channel.has_value());
    build(config);
    ASSERT_TRUE(transmitter_->initialize());

    std::array<int16_t, kMaxChannels> channels{};
    channels.fill(kPressed);
    feed_channels(channels);
    EXPECT_FALSE(transmitter_->did_init_button_press());
}

TEST_F(OpenTxTransmitterTest, EveryOpenWritesThePrimingCommands) {
    build(button_config());
    ASSERT_TRUE(transmitter_->initialize());
    const std::string first = read_master();
    EXPECT_NE(first.find("telemetry on\r\n"), std::string::npos);
    EXPECT_NE(first.find("channels on\r\n"), std::string::npos);

    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    transmitter_->update();
    const std::string reopened = read_master();
    EXPECT_NE(reopened.find("telemetry on\r\n"), std::string::npos)
        << "a reopened port was not primed";
    EXPECT_NE(reopened.find("channels on\r\n"), std::string::npos);
}

TEST_F(OpenTxTransmitterTest, FreshOpenGetsAGraceWindowBeforeReadingAsStalled) {
    build(button_config());
    ASSERT_TRUE(transmitter_->initialize());

    const TransmitterStatus fresh = transmitter_->get_status();
    EXPECT_TRUE(fresh.connected);
    EXPECT_TRUE(fresh.receiving_channels) << "a healthy open flashed a stalled channel stream";

    std::this_thread::sleep_for(std::chrono::milliseconds(650));
    EXPECT_FALSE(transmitter_->get_status().receiving_channels);
}

TEST_F(OpenTxTransmitterTest, FirstChannelFrameLogsSwitchStatesEvenWhenAllOff) {
    build(button_config());
    ASSERT_TRUE(transmitter_->initialize());

    feed(kReleased, kReleased);

    const auto snapshots = DiagnosticsLogger::get_logger("opentx_transmitter")->get_snapshots();
    const auto found = std::find_if(snapshots.begin(), snapshots.end(), [](const auto &snapshot) {
        return snapshot.subsection == "switch_states";
    });
    ASSERT_NE(found, snapshots.end()) << "no switch_states section after the first frame";
    EXPECT_EQ(std::get<int>(found->values.at("has_switch_states")), 1);
    EXPECT_EQ(std::get<int>(found->values.at("autonomy_enabled")), 0);
    EXPECT_EQ(std::get<int>(found->values.at("init_button")), 0);
}

TEST_F(OpenTxTransmitterTest, TrainerEngageRampsFromStandstill) {
    build(limited_config());
    ASSERT_TRUE(transmitter_->initialize());
    transmitter_->enable();
    read_master();

    feed(kReleased, kPressed);
    clock_->set(0.0);
    transmitter_->send({1.0, 0.0, 0.0});
    for (int step = 1; step <= 10; ++step) {
        clock_->set(kStepS * step);
        transmitter_->send({1.0, 0.0, 0.0});
    }
    const auto saturated = last_linear_trainer_value(read_master());
    ASSERT_TRUE(saturated.has_value());

    feed(kReleased, kReleased);
    clock_->set(1.2);
    transmitter_->send({1.0, 0.0, 0.0});
    read_master();

    feed(kReleased, kPressed);
    clock_->set(1.3);
    transmitter_->send({1.0, 0.0, 0.0});
    const auto engaged = last_linear_trainer_value(read_master());
    ASSERT_TRUE(engaged.has_value());
    EXPECT_LT(*engaged, *saturated / 2) << "engaging the trainer resumed the old command";
    EXPECT_GT(*engaged, 0);
}

TEST_F(OpenTxTransmitterTest, ReEnablingStartsFromStandstill) {
    build(limited_config());
    ASSERT_TRUE(transmitter_->initialize());
    feed(kReleased, kPressed);
    transmitter_->enable();
    clock_->set(0.0);
    transmitter_->send({1.0, 0.0, 0.0});
    for (int step = 1; step <= 10; ++step) {
        clock_->set(kStepS * step);
        transmitter_->send({1.0, 0.0, 0.0});
    }
    read_master();
    clock_->set(1.1);
    transmitter_->send({1.0, 0.0, 0.0});
    const auto saturated = last_linear_trainer_value(read_master());
    ASSERT_TRUE(saturated.has_value());

    transmitter_->disable();
    transmitter_->enable();
    clock_->set(1.2);
    transmitter_->send({0.0, 0.0, 0.0});
    const auto resumed = last_linear_trainer_value(read_master());
    ASSERT_TRUE(resumed.has_value());
    EXPECT_EQ(*resumed, 0) << "re-enabling slewed down from the command sent before disable";
}

}  // namespace
}  // namespace auto_battlebot
