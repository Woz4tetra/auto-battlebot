#include <espnow_protocol.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <pty.h>
#include <sys/inotify.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transmitter/config.hpp"
#include "transmitter/espnow_transmitter.hpp"

namespace auto_battlebot {
TEST(EspNowTelemetryJsonTest, NonFiniteFloatsAndMissingCommandBecomeNull) {
    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.timestamp_ms = 1234;
    telemetry.a_percent = 12.5f;
    telemetry.pack_volts = NAN;
    telemetry.armed = 1;
    telemetry.espnow_age_ms = UINT32_MAX;
    telemetry.telemetry_seq = 30517;
    telemetry.gyro_y_dps = -2.5f;
    constexpr uint8_t kCalibSys = 3, kCalibGyro = 2, kCalibAccel = 1, kCalibMag = 0;
    telemetry.imu_calib_stat =
        (kCalibSys << 6) | (kCalibGyro << 4) | (kCalibAccel << 2) | kCalibMag;
    telemetry.motor_amps_est_left = 6.5f;
    telemetry.motor_speed_right_rad_s = NAN;
    telemetry.guard_state = 3;
    telemetry.stall_flags = 2;
    telemetry.erpm_crc_errors = 517;
    const std::string json = to_espnow_telemetry_json(telemetry, 99, std::nullopt);
    EXPECT_NE(json.find("\"host_receive_ns\":99"), std::string::npos);
    EXPECT_NE(json.find("\"a_percent\":12.5"), std::string::npos);
    EXPECT_NE(json.find("\"vbat\":null"), std::string::npos);
    EXPECT_NE(json.find("\"armed\":true"), std::string::npos);
    EXPECT_NE(json.find("\"espnow_age_ms\":null"), std::string::npos);
    EXPECT_NE(json.find("\"round_trip_us\":null"), std::string::npos);
    EXPECT_NE(json.find("\"telemetry_seq\":30517"), std::string::npos);
    EXPECT_NE(json.find("\"gyro_y\":-2.5"), std::string::npos);
    EXPECT_NE(json.find("\"imu_calib_sys\":3"), std::string::npos);
    EXPECT_NE(json.find("\"imu_calib_gyro\":2"), std::string::npos);
    EXPECT_NE(json.find("\"imu_calib_accel\":1"), std::string::npos);
    EXPECT_NE(json.find("\"imu_calib_mag\":0"), std::string::npos);
    EXPECT_NE(json.find("\"motor_amps_left\":6.5"), std::string::npos);
    EXPECT_NE(json.find("\"motor_speed_right\":null"), std::string::npos);
    EXPECT_NE(json.find("\"guard_state\":3"), std::string::npos);
    EXPECT_NE(json.find("\"stall_flags\":2"), std::string::npos);
    EXPECT_NE(json.find("\"erpm_crc_errors\":517,\"round_trip_us\":null"), std::string::npos);
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json.back(), '}');
}

class EspNowTransmitterPtyTest : public ::testing::Test {
   protected:
    void SetUp() override {
        char name[256];
        ASSERT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
        path_ = name;
        fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK);
    }
    void TearDown() override {
        close(master_);
        close(slave_);
    }

    std::vector<uint8_t> read_master() {
        std::vector<uint8_t> bytes;
        uint8_t buffer[512];
        for (int i = 0; i < 50; i++) {
            const ssize_t n = read(master_, buffer, sizeof(buffer));
            if (n > 0) {
                bytes.insert(bytes.end(), buffer, buffer + n);
            } else if (!bytes.empty()) {
                break;
            } else {
                usleep(1000);
            }
        }
        return bytes;
    }

    int master_ = -1;
    int slave_ = -1;
    std::string path_;
};

struct ParsedFrame {
    uint8_t type;
    std::vector<uint8_t> payload;
};

void collect_frame(void *context, uint8_t type, const uint8_t *payload, uint8_t length) {
    static_cast<std::vector<ParsedFrame> *>(context)->push_back(
        {type, std::vector<uint8_t>(payload, payload + length)});
}

TEST_F(EspNowTransmitterPtyTest, SendsProcessedCommandsWithIncreasingSeqOnlyWhileEnabled) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    config.reverse_angular_channel = true;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    ASSERT_TRUE(transmitter.initialize());

    transmitter.send({0.5, 0.0, 0.25});
    EXPECT_TRUE(read_master().empty());

    transmitter.enable();
    transmitter.send({0.5, 0.0, 0.25});
    transmitter.send({0.25, 0.0, 0.0});
    const std::vector<uint8_t> bytes = read_master();

    espnow_protocol::SerialFrameParser parser;
    std::vector<ParsedFrame> frames;
    parser.feed(bytes.data(), bytes.size(), collect_frame, &frames);
    ASSERT_EQ(frames.size(), 2u);
    ASSERT_EQ(frames[0].type, espnow_protocol::MSG_COMMAND);
    ASSERT_EQ(frames[0].payload.size(), sizeof(espnow_protocol::CommandPayload));
    espnow_protocol::CommandPayload first, second;
    std::memcpy(&first, frames[0].payload.data(), sizeof(first));
    std::memcpy(&second, frames[1].payload.data(), sizeof(second));
    EXPECT_EQ(second.seq, first.seq + 1);
    EXPECT_FLOAT_EQ(first.ch0_value, 0.5f);
    EXPECT_FLOAT_EQ(first.ch1_value, -0.25f) << "reversed, same as the OpenTX child";
    EXPECT_FLOAT_EQ(second.ch0_value, 0.25f);
}

bool logged_silent_reconnect() {
    for (const auto &snapshot :
         DiagnosticsLogger::get_logger("espnow_transmitter")->get_snapshots()) {
        if (snapshot.subsection == "dongle_silent_reconnecting" &&
            !snapshot.is_repeat_of_earlier_message)
            return true;
    }
    return false;
}

TEST(EspNowTransmitterReopenTest, ReopenAttemptsAreRateLimited) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "auto_battlebot_dongle_probe";
    std::ofstream(path).put('x');
    const int watch_fd = inotify_init1(IN_NONBLOCK);
    ASSERT_GE(watch_fd, 0);
    ASSERT_GE(inotify_add_watch(watch_fd, path.c_str(), IN_OPEN), 0);

    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path.string();
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    int opens = 0;
    const auto drain_opens = [&] {
        alignas(inotify_event) char buffer[4096];
        ssize_t n;
        while ((n = read(watch_fd, buffer, sizeof(buffer))) > 0) {
            for (ssize_t offset = 0; offset < n;) {
                const auto *event = reinterpret_cast<const inotify_event *>(buffer + offset);
                if (event->mask & IN_OPEN) ++opens;
                offset += static_cast<ssize_t>(sizeof(inotify_event) + event->len);
            }
        }
    };
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1800)) {
        transmitter.update();
        drain_opens();
        usleep(10000);
    }

    close(watch_fd);
    std::filesystem::remove(path);
    EXPECT_GE(opens, 1);
    EXPECT_LE(opens, 2);
}

TEST_F(EspNowTransmitterPtyTest, SilentDongleIsNotClosedBeforeTheReopenIntervalElapses) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    auto logger = DiagnosticsLogger::get_logger("espnow_transmitter");
    ASSERT_TRUE(transmitter.initialize());
    logger->clear();

    const auto run_for = [&](std::chrono::milliseconds duration) {
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < duration) {
            transmitter.update();
            usleep(10000);
        }
    };
    run_for(std::chrono::milliseconds(800));
    EXPECT_FALSE(logged_silent_reconnect());
    run_for(std::chrono::milliseconds(500));
    EXPECT_TRUE(logged_silent_reconnect());
}

TEST_F(EspNowTransmitterPtyTest, ReopensTheDongleOnlyAfterItGoesSilent) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    auto logger = DiagnosticsLogger::get_logger("espnow_transmitter");
    ASSERT_TRUE(transmitter.initialize());
    logger->clear();

    const espnow_protocol::LinkStatsPayload stats = {};
    uint8_t frame[espnow_protocol::SERIAL_MAX_FRAME];
    const size_t n = espnow_protocol::encode_serial_frame(espnow_protocol::MSG_LINK_STATS,
                                                          reinterpret_cast<const uint8_t *>(&stats),
                                                          sizeof(stats), frame);
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1200)) {
        ASSERT_EQ(write(master_, frame, n), static_cast<ssize_t>(n));
        usleep(100000);
        transmitter.update();
    }
    EXPECT_FALSE(logged_silent_reconnect());

    usleep(1100000);
    transmitter.update();
    EXPECT_TRUE(logged_silent_reconnect());
}

namespace {

void write_telemetry(int fd, const espnow_protocol::TelemetryPayload &telemetry) {
    uint8_t frame[espnow_protocol::SERIAL_MAX_FRAME];
    const size_t n = espnow_protocol::encode_serial_frame(
        espnow_protocol::MSG_TELEMETRY, reinterpret_cast<const uint8_t *>(&telemetry),
        sizeof(telemetry), frame);
    ASSERT_EQ(write(fd, frame, n), static_cast<ssize_t>(n));
}

}  // namespace

std::vector<espnow_protocol::CommandPayload> read_commands(const std::vector<uint8_t> &bytes) {
    espnow_protocol::SerialFrameParser parser;
    std::vector<ParsedFrame> frames;
    parser.feed(bytes.data(), bytes.size(), collect_frame, &frames);
    std::vector<espnow_protocol::CommandPayload> commands;
    for (const auto &frame : frames) {
        if (frame.type != espnow_protocol::MSG_COMMAND) continue;
        espnow_protocol::CommandPayload command;
        std::memcpy(&command, frame.payload.data(), sizeof(command));
        commands.push_back(command);
    }
    return commands;
}

TEST_F(EspNowTransmitterPtyTest, SlewLimitsLinearOnlyWhileAutonomyIsOn) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    config.max_motor_rpm = 1500.0;
    config.max_motor_rpm_per_s = config.max_motor_rpm;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    ASSERT_TRUE(transmitter.initialize());
    transmitter.enable();

    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.espnow_age_ms = UINT32_MAX;
    telemetry.radio_connected = 1;
    telemetry.autonomy_switch_sa = 0;
    write_telemetry(master_, telemetry);
    usleep(5000);
    transmitter.update();
    const auto last_sa_off_time = std::chrono::steady_clock::now();
    transmitter.send({1.0, 0.0, 0.5});
    auto commands = read_commands(read_master());
    ASSERT_EQ(commands.size(), 1u);
    EXPECT_FLOAT_EQ(commands[0].ch0_value, 0.5f) << "SA off: unlimited, saturation splits 1.0";

    telemetry.autonomy_switch_sa = 1;
    write_telemetry(master_, telemetry);
    usleep(5000);
    transmitter.update();
    usleep(50000);
    transmitter.send({1.0, 0.0, 0.0});
    const double elapsed_since_sa_off_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_sa_off_time).count();
    commands = read_commands(read_master());
    ASSERT_EQ(commands.size(), 1u);
    EXPECT_GT(commands[0].ch0_value, 0.0f);
    EXPECT_LE(commands[0].ch0_value, elapsed_since_sa_off_s + 1e-3);
}

TEST_F(EspNowTransmitterPtyTest, ReEnablingWithSaOnRampsFromStandstill) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    config.max_motor_rpm = 1500.0;
    config.max_motor_rpm_per_s = config.max_motor_rpm;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    ASSERT_TRUE(transmitter.initialize());

    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.espnow_age_ms = UINT32_MAX;
    telemetry.radio_connected = 1;
    telemetry.autonomy_switch_sa = 1;
    write_telemetry(master_, telemetry);
    usleep(5000);
    transmitter.update();

    transmitter.enable();
    transmitter.send({1.0, 0.0, 0.0});
    usleep(50000);
    transmitter.disable();
    transmitter.enable();
    const auto enabled_at = std::chrono::steady_clock::now();
    usleep(50000);
    write_telemetry(master_, telemetry);
    usleep(5000);
    transmitter.update();
    ASSERT_TRUE(transmitter.get_status().autonomy_switch_on);
    transmitter.send({1.0, 0.0, 0.0});
    const double since_enable_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - enabled_at).count();
    const auto commands = read_commands(read_master());
    ASSERT_FALSE(commands.empty());
    EXPECT_LE(commands.back().ch0_value, since_enable_s + 1e-3)
        << "the first command after enable must not skip the slew limit";
}

TEST(EspNowTelemetryChannelsTest, UndoesTheFirmwareNegationAndTheUpsideDownFlip) {
    espnow_protocol::TelemetryPayload upright = {};
    upright.a_percent = -40.0f;
    upright.b_percent = 25.0f;
    auto channels = espnow_telemetry_channels(upright);
    EXPECT_NEAR(channels.first_output, 0.4, 1e-6);
    EXPECT_NEAR(channels.second_output, 0.25, 1e-6);

    espnow_protocol::TelemetryPayload flipped = upright;
    flipped.a_percent = 40.0f;
    flipped.is_upside_down = 1;
    channels = espnow_telemetry_channels(flipped);
    EXPECT_NEAR(channels.first_output, 0.4, 1e-6);
    EXPECT_NEAR(channels.second_output, 0.25, 1e-6);
}

TEST(EspNowRoundTripTest, RoundTripAcrossClockWrap) {
    EXPECT_EQ(espnow_round_trip_us(1500, 1000), 500);
    EXPECT_EQ(espnow_round_trip_us(5, 0xFFFFFFF0u), 21);
}

TEST_F(EspNowTransmitterPtyTest, NoFeedbackOrSwitchBeforeTelemetry) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    ASSERT_TRUE(transmitter.initialize());

    const char *text = "\nespnow_dongle mac=d8:3b channel=1\n";
    ASSERT_GT(write(master_, text, std::strlen(text)), 0);
    usleep(5000);
    EXPECT_TRUE(transmitter.update().stick_commands.empty());
    const TransmitterStatus status = transmitter.get_status();
    EXPECT_TRUE(status.connected);
    EXPECT_FALSE(status.receiving_channels);
    EXPECT_TRUE(status.has_trainer_autonomy_switch);
    EXPECT_FALSE(status.autonomy_switch_on);
}

TEST_F(EspNowTransmitterPtyTest, FeedbackAndSwitchComeFromTelemetry) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    config.reverse_angular_channel = true;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    ASSERT_TRUE(transmitter.initialize());

    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.espnow_age_ms = UINT32_MAX;
    telemetry.a_percent = -50.0f;
    telemetry.b_percent = 20.0f;
    telemetry.radio_connected = 1;
    telemetry.autonomy_switch_sa = 1;
    write_telemetry(master_, telemetry);
    usleep(5000);

    const auto processor = make_drive_processor(
        config.drive_processor, {.reverse_angular = config.reverse_angular_channel},
        DiagnosticsLogger::get_logger("espnow_feedback_test"));
    const BodyVelocity expected = processor->to_body_velocity({0.5, 0.2});
    const CommandFeedback feedback = transmitter.update();
    ASSERT_EQ(feedback.stick_commands.count(FrameId::OUR_ROBOT_1), 1u);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).forward, expected.linear, 1e-6);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).yaw, expected.angular, 1e-6);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).forward, 0.5, 1e-6);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).yaw,
                (config.reverse_angular_channel ? -1.0 : 1.0) * 0.2, 1e-6);
    EXPECT_TRUE(transmitter.get_status().receiving_channels);
    EXPECT_TRUE(transmitter.get_status().autonomy_switch_on);

    telemetry.autonomy_switch_sa = 0;
    write_telemetry(master_, telemetry);
    usleep(5000);
    transmitter.update();
    EXPECT_TRUE(transmitter.get_status().receiving_channels);
    EXPECT_FALSE(transmitter.get_status().autonomy_switch_on);

    telemetry.autonomy_switch_sa = 1;
    telemetry.radio_connected = 0;
    write_telemetry(master_, telemetry);
    usleep(5000);
    transmitter.update();
    EXPECT_FALSE(transmitter.get_status().receiving_channels);
    EXPECT_FALSE(transmitter.get_status().autonomy_switch_on);

    usleep(std::chrono::duration_cast<std::chrono::microseconds>(kTelemetryStale).count() + 20000);
    EXPECT_TRUE(transmitter.update().stick_commands.empty());
    EXPECT_FALSE(transmitter.get_status().receiving_channels);
}

}  // namespace auto_battlebot
