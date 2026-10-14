#include <espnow_protocol.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <pty.h>
#include <toml++/toml.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transmitter/composite_transmitter.hpp"
#include "transmitter/config.hpp"
#include "transmitter/espnow_transmitter.hpp"

namespace auto_battlebot {
namespace {

/** Records what the composite does to it, and reports whatever the test sets. */
class FakeTransmitter : public TransmitterInterface {
   public:
    bool init_result = true;
    bool button = false;
    TransmitterStatus status;
    CommandFeedback feedback;
    int initialize_calls = 0;
    int update_calls = 0;
    int button_polls = 0;
    std::vector<VelocityCommand> sent;
    bool enabled = false;

    bool initialize() override {
        initialize_calls++;
        return init_result;
    }
    CommandFeedback update() override {
        update_calls++;
        return feedback;
    }
    void send(VelocityCommand command) override { sent.push_back(command); }
    bool did_init_button_press() override {
        button_polls++;
        return button;
    }
    TransmitterStatus get_status() const override { return status; }
    void enable() override { enabled = true; }
    void disable() override { enabled = false; }
};

std::unique_ptr<TransmitterConfiguration> parse(const std::string &text) {
    const toml::table table = toml::parse(text);
    ConfigParser parser(*table["transmitter"].as_table(), "transmitter");
    return parse_transmitter_config(parser);
}

const char *kCompositeToml = R"(
[transmitter]
type = "CompositeTransmitter"

[[transmitter.children]]
type = "OpenTxTransmitter"
transmit_rate_hz = 150
drive_processor = "DifferentialDriveProcessor"
zero_deadzone_percent = 2
reverse_angular_channel = true

[[transmitter.children]]
type = "EspNowTransmitter"
dongle_serial_path = "/dev/null-dongle"
drive_processor = "DifferentialDriveProcessor"
zero_deadzone_percent = 2
reverse_angular_channel = true
)";

}  // namespace

TEST(CompositeTransmitterTest, PrimaryInitializeDecidesAndAcceleratorFailureIsIgnored) {
    auto primary = std::make_shared<FakeTransmitter>();
    auto accelerator = std::make_shared<FakeTransmitter>();
    accelerator->init_result = false;
    CompositeTransmitter composite({primary, accelerator});
    EXPECT_TRUE(composite.initialize());
    EXPECT_EQ(accelerator->initialize_calls, 1);

    primary->init_result = false;
    accelerator->init_result = true;
    EXPECT_FALSE(composite.initialize());
}

TEST(CompositeTransmitterTest, FansOutSendEnableAndDisable) {
    auto a = std::make_shared<FakeTransmitter>();
    auto b = std::make_shared<FakeTransmitter>();
    CompositeTransmitter composite({a, b});
    composite.enable();
    composite.send({0.5, 0.0, -0.25});
    ASSERT_EQ(a->sent.size(), 1u);
    ASSERT_EQ(b->sent.size(), 1u);
    EXPECT_DOUBLE_EQ(b->sent[0].angular_z, -0.25);
    EXPECT_TRUE(a->enabled && b->enabled);
    composite.disable();
    EXPECT_FALSE(a->enabled || b->enabled);
}

TEST(CompositeTransmitterTest, CapsAChildsRateAndDeliversTheCommandItSkipped) {
    auto trainer = std::make_shared<FakeTransmitter>();
    auto espnow = std::make_shared<FakeTransmitter>();
    std::chrono::steady_clock::time_point now{};
    CompositeTransmitter composite({trainer, espnow}, {100.0, 0.0}, [&] { return now; });

    composite.send({0.1, 0.0, 0.0});
    now += std::chrono::milliseconds(4);
    composite.send({0.2, 0.0, 0.0});  // inside the trainer's 10 ms period
    EXPECT_EQ(trainer->sent.size(), 1u);
    EXPECT_EQ(espnow->sent.size(), 2u);

    now += std::chrono::milliseconds(6);
    composite.update();  // period over, no newer send: the skipped command goes out
    ASSERT_EQ(trainer->sent.size(), 2u);
    EXPECT_DOUBLE_EQ(trainer->sent[1].linear_x, 0.2);
    composite.update();
    EXPECT_EQ(trainer->sent.size(), 2u);  // delivered once
}

TEST(CompositeTransmitterTest, DisableFlushesTheFinalStopToACappedChild) {
    auto trainer = std::make_shared<FakeTransmitter>();
    std::chrono::steady_clock::time_point now{};
    CompositeTransmitter composite({trainer}, {100.0}, [&] { return now; });
    composite.send({0.5, 0.0, 0.0});
    now += std::chrono::milliseconds(1);
    composite.send({0.0, 0.0, 0.0});
    composite.disable();
    ASSERT_EQ(trainer->sent.size(), 2u);
    EXPECT_DOUBLE_EQ(trainer->sent.back().linear_x, 0.0);
}

TEST(CompositeTransmitterTest, PrimaryWinsFeedbackCollisionsAndEveryChildIsPolled) {
    auto primary = std::make_shared<FakeTransmitter>();
    auto other = std::make_shared<FakeTransmitter>();
    primary->feedback.stick_commands[FrameId::OUR_ROBOT_1] = {1.0, 0.0, 0.0};
    other->feedback.stick_commands[FrameId::OUR_ROBOT_1] = {-1.0, 0.0, 0.0};
    CompositeTransmitter composite({primary, other});
    const CommandFeedback merged = composite.update();
    EXPECT_DOUBLE_EQ(merged.stick_commands.at(FrameId::OUR_ROBOT_1).linear_x, 1.0);
    EXPECT_EQ(other->update_calls, 1);
}

TEST(CompositeTransmitterTest, ButtonPollsEveryChildAndStatusComesFromPrimary) {
    auto primary = std::make_shared<FakeTransmitter>();
    auto other = std::make_shared<FakeTransmitter>();
    primary->button = true;
    primary->status.connected = true;
    CompositeTransmitter composite({primary, other});
    EXPECT_TRUE(composite.did_init_button_press());
    EXPECT_EQ(other->button_polls, 1);  // no short-circuit, so its edge latch clears
    EXPECT_TRUE(composite.get_status().connected);
}

TEST(CompositeTransmitterConfigTest, ParsesChildrenInOrder) {
    const auto config = parse(kCompositeToml);
    ASSERT_EQ(config->type, "CompositeTransmitter");
    const auto &composite = static_cast<const CompositeTransmitterConfiguration &>(*config);
    ASSERT_EQ(composite.children.size(), 2u);
    EXPECT_EQ(composite.children[0]->type, "OpenTxTransmitter");
    EXPECT_EQ(composite.children[1]->type, "EspNowTransmitter");
    EXPECT_EQ(composite.transmit_rate_hz, (std::vector<double>{150.0, 0.0}));
    const auto &espnow =
        static_cast<const EspNowTransmitterConfiguration &>(*composite.children[1]);
    EXPECT_EQ(espnow.dongle_serial_path, "/dev/null-dongle");
    EXPECT_TRUE(espnow.reverse_angular_channel);
}

TEST(CompositeTransmitterConfigTest, RefusesProcessorSettingsThatDisagree) {
    std::string text = kCompositeToml;
    // Drop the ESP-NOW child's reversal: the transports would disagree on the turn direction.
    const std::string line = "reverse_angular_channel = true\n";
    text.erase(text.rfind(line), line.size());
    EXPECT_THROW(parse(text), ConfigValidationError);
}

TEST(CompositeTransmitterConfigTest, DriveProcessorDefaultsToDifferentialAndMustMatch) {
    std::string text = kCompositeToml;
    const std::string line = "drive_processor = \"DifferentialDriveProcessor\"\n";
    text.erase(text.rfind(line), line.size());
    const auto config = parse(text);  // the default matches the OpenTX child
    const auto &composite = static_cast<const CompositeTransmitterConfiguration &>(*config);
    EXPECT_EQ(
        static_cast<const EspNowTransmitterConfiguration &>(*composite.children[1]).drive_processor,
        "DifferentialDriveProcessor");

    std::string tank = kCompositeToml;
    tank.replace(tank.rfind("DifferentialDriveProcessor"), 26, "TankDriveProcessor");
    EXPECT_THROW(parse(tank), ConfigValidationError);
}

TEST(CompositeTransmitterConfigTest, RefusesNestingAndEmptyChildren) {
    EXPECT_THROW(parse(R"(
[transmitter]
type = "CompositeTransmitter"
[[transmitter.children]]
type = "CompositeTransmitter"
[[transmitter.children.children]]
type = "NoopTransmitter"
)"),
                 ConfigValidationError);
    EXPECT_THROW(parse(R"(
[transmitter]
type = "CompositeTransmitter"
children = []
)"),
                 ConfigValidationError);
}

TEST(EspNowTelemetryJsonTest, NonFiniteFloatsAndMissingCommandBecomeNull) {
    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.timestamp_ms = 1234;
    telemetry.a_percent = 12.5f;
    telemetry.vbat = NAN;
    telemetry.armed = 1;
    telemetry.espnow_age_ms = UINT32_MAX;
    telemetry.telemetry_seq = 30517;
    telemetry.gyro_y = -2.5f;
    telemetry.imu_calibration = 0b11100100;  // sys 3, gyro 2, accel 1, mag 0
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
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json.back(), '}');
}

/** End to end through a pseudo-terminal standing in for the dongle's USB serial port. */
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
    EXPECT_TRUE(read_master().empty());  // disabled: nothing on the wire

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
    EXPECT_FLOAT_EQ(first.channel_a, 0.5f);
    EXPECT_FLOAT_EQ(first.channel_b, -0.25f);  // reversed, same as the OpenTX child would
    EXPECT_FLOAT_EQ(second.channel_a, 0.25f);
}

bool logged_silent_reconnect() {
    for (const auto &snapshot :
         DiagnosticsLogger::get_logger("espnow_transmitter")->get_snapshots()) {
        if (snapshot.subsection == "dongle_silent_reconnecting" && !snapshot.held) return true;
    }
    return false;
}

// A dongle that re-enumerates leaves the old fd open and reading empty forever. Silence past one
// reconnect interval must drop and reopen the port; link stats arriving must not.
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

TEST_F(EspNowTransmitterPtyTest, DrainsTelemetryAndLinkStatsWithoutFeedback) {
    EspNowTransmitterConfiguration config;
    config.dongle_serial_path = path_;
    EspNowTransmitter transmitter(config, nullptr, nullptr);
    ASSERT_TRUE(transmitter.initialize());

    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.espnow_age_ms = UINT32_MAX;
    uint8_t frame[espnow_protocol::SERIAL_MAX_FRAME];
    const char *text = "\nespnow_dongle mac=d8:3b channel=1\n";
    ASSERT_GT(write(master_, text, std::strlen(text)), 0);
    const size_t n = espnow_protocol::encode_serial_frame(
        espnow_protocol::MSG_TELEMETRY, reinterpret_cast<const uint8_t *>(&telemetry),
        sizeof(telemetry), frame);
    ASSERT_EQ(write(master_, frame, n), static_cast<ssize_t>(n));
    usleep(5000);
    EXPECT_TRUE(transmitter.update().stick_commands.empty());
}

}  // namespace auto_battlebot
