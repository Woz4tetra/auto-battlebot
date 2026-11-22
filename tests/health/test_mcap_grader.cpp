#include <gtest/gtest.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "health/mcap_grader.hpp"

namespace auto_battlebot {
namespace {

using nlohmann::json;

class RecordingWriter {
   public:
    explicit RecordingWriter(const std::filesystem::path &path) {
        const auto status = writer_.open(path.string(), mcap::McapWriterOptions("test"));
        EXPECT_TRUE(status.ok()) << status.message;
    }

    void write(const std::string &topic, double t, const json &payload) {
        auto [it, inserted] = channels_.try_emplace(topic, 0);
        if (inserted) {
            mcap::Channel channel(topic, "json", 0);
            writer_.addChannel(channel);
            it->second = channel.id;
        }
        const std::string text = payload.dump();
        mcap::Message message;
        message.channelId = it->second;
        message.sequence = sequence_++;
        message.logTime = kStartNs + static_cast<uint64_t>(t * 1e9);
        message.publishTime = message.logTime;
        message.data = reinterpret_cast<const std::byte *>(text.data());
        message.dataSize = text.size();
        const auto status = writer_.write(message);
        EXPECT_TRUE(status.ok()) << status.message;
    }

    void close() { writer_.close(); }

   private:
    static constexpr uint64_t kStartNs = 1'790'000'000'000'000'000ULL;
    mcap::McapWriter writer_;
    std::map<std::string, mcap::ChannelId> channels_;
    uint32_t sequence_ = 0;
};

struct Session {
    std::string transmitter = "OpenTxTransmitter";
    bool crsf_telemetry = true;
    int outside_every = 0;
    double opponent_sigma_m = 0.05;
    std::optional<int> guard_state;
};

json diagnostics(const std::string &subsection, const json &values) {
    return {{subsection, {{"level", 0}, {"message", ""}, {"held", false}, {"values", values}}}};
}

std::filesystem::path write_session(const std::string &name, const Session &session) {
    const auto path = std::filesystem::temp_directory_path() /
                      ("mcap_grader_" + std::to_string(getpid()) + "_" + name + ".mcap");
    RecordingWriter writer(path);
    writer.write("/status/config", 0.0,
                 {{"profile", "test"},
                  {"fields",
                   {{{"section", "transmitter"},
                     {"key", "type"},
                     {"value", "'" + session.transmitter + "'"},
                     {"source", "test.toml:1"}}}}});
    int tracks = 0;
    for (int i = 0; i <= 20 * 60; ++i) {
        const double t = i / 60.0;
        if (i % 2 == 0) {
            writer.write("/camera/frame_meta", t, {{"video_frame_index", i / 2}});
            writer.write("/keypoint_detections", t, json::object());
        }
        if (i % 2 == 0) {
            writer.write(
                "/status/system", t,
                {{"initialized", true}, {"autonomy_enabled", true}, {"autonomy_switch_on", true}});
            json opentx = diagnostics("send", {{"channel_a_val", 0}});
            if (session.crsf_telemetry) {
                opentx.update(diagnostics("link_statistics", {{"up_lq", 100}}));
            }
            if (session.transmitter == "OpenTxTransmitter") {
                writer.write("/diagnostics/opentx_transmitter", t, opentx);
            }
            writer.write("/diagnostics/kalman_motion_estimator", t,
                         diagnostics("opponent_NEUTRAL_ROBOT_1",
                                     {{"position_sigma_major_axis_m", session.opponent_sigma_m}}));
        }
        if (session.transmitter == "EspNowTransmitter") {
            json telemetry = {{"yaw_rate", 0.0},
                              {"espnow_seq", i},
                              {"espnow_age_ms", 5},
                              {"guard_scale", 1.0},
                              {"guard_state", session.guard_state.value_or(1)},
                              {"espnow_active", false},
                              {"is_upside_down", false}};
            writer.write("/robot/espnow_telemetry", t, telemetry);
        }
        if (i % 3 == 0) {
            const bool outside = session.outside_every > 0 && tracks % session.outside_every == 0;
            ++tracks;
            writer.write("/status/tracks", t,
                         {{"field_x", 1.5},
                          {"field_y", 1.5},
                          {"robots",
                           {{{"id", "our_robot_1"},
                             {"ours", true},
                             {"x", outside ? 0.9 : 0.0},
                             {"y", 0.0},
                             {"yaw", 0.0}},
                            {{"id", "neutral_robot_1"},
                             {"ours", false},
                             {"x", 0.5},
                             {"y", 0.5},
                             {"yaw", 0.0}}}}});
        }
    }
    writer.close();
    return path;
}

const PerformanceWarning *find(const McapGrade &grade, PerformanceCheck check) {
    for (const auto &warning : grade.warnings) {
        if (warning.check == check) return &warning;
    }
    return nullptr;
}

TEST(McapGraderTest, HealthySessionGradesOk) {
    const auto path = write_session("healthy", Session{});
    const McapGrade grade = grade_mcap(path, PerformanceMonitorConfiguration{});
    std::filesystem::remove(path);
    EXPECT_EQ(grade.transmitter_type, "OpenTxTransmitter");
    EXPECT_NEAR(grade.duration_s, 20.0, 0.01);
    EXPECT_TRUE(grade.warnings.empty()) << format_grade(grade);
    EXPECT_FALSE(grade.red());
}

TEST(McapGraderTest, CrsfSilenceCalibrationAndPhantomAllFireOffline) {
    Session session;
    session.crsf_telemetry = false;
    session.outside_every = 12;
    session.opponent_sigma_m = 122.0;
    const auto path = write_session("broken", session);
    const McapGrade grade = grade_mcap(path, PerformanceMonitorConfiguration{});
    std::filesystem::remove(path);
    const std::string report = format_grade(grade);

    const PerformanceWarning *crsf = find(grade, PerformanceCheck::CRSF_TELEMETRY_LINK);
    ASSERT_NE(crsf, nullptr) << report;
    EXPECT_EQ(crsf->severity, WarningSeverity::RED);
    EXPECT_TRUE(grade.red());

    const PerformanceWarning *calibration = find(grade, PerformanceCheck::CALIBRATION);
    ASSERT_NE(calibration, nullptr) << report;
    EXPECT_NEAR(calibration->values.at("fraction"), 1.0 / 12.0, 0.02);

    const PerformanceWarning *phantom = find(grade, PerformanceCheck::PHANTOM_TRACK);
    ASSERT_NE(phantom, nullptr) << report;
    EXPECT_NE(phantom->message.find("neutral_robot_1"), std::string::npos);
    EXPECT_NE(report.find("verdict: RED"), std::string::npos);
}

TEST(McapGraderTest, ThermalTripInEspNowTelemetryIsRed) {
    Session session;
    session.transmitter = "EspNowTransmitter";
    session.guard_state = kGuardStateTripped;
    const auto path = write_session("tripped", session);
    const McapGrade grade = grade_mcap(path, PerformanceMonitorConfiguration{});
    std::filesystem::remove(path);
    const PerformanceWarning *trip = find(grade, PerformanceCheck::THERMAL_TRIP);
    ASSERT_NE(trip, nullptr) << format_grade(grade);
    EXPECT_EQ(trip->severity, WarningSeverity::RED);
    EXPECT_EQ(find(grade, PerformanceCheck::ESPNOW_LINK), nullptr) << format_grade(grade);
    EXPECT_EQ(find(grade, PerformanceCheck::IMU_LINK), nullptr) << format_grade(grade);
}

}  // namespace
}  // namespace auto_battlebot

namespace auto_battlebot {
namespace {

std::filesystem::path saved_recording(const std::string &relative) {
    return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "data" /
           "saved_recordings" / relative;
}

bool has_crsf_warning(const McapGrade &grade) {
    for (const auto &warning : grade.warnings) {
        if (warning.check == PerformanceCheck::CRSF_TELEMETRY_LINK) return true;
    }
    return false;
}

TEST(McapGraderRecordedTest, MayNhrlEdgeTxSessionHasCrsfTelemetry) {
    const auto path =
        saved_recording("NHRL_2026-05-02/auto_battlebot_main_2026-05-02_10-06-02_repaired.mcap");
    if (!std::filesystem::exists(path)) GTEST_SKIP() << "recording not on this machine: " << path;
    const McapGrade grade = grade_mcap(path, PerformanceMonitorConfiguration{});
    EXPECT_FALSE(has_crsf_warning(grade)) << format_grade(grade);
    EXPECT_GT(grade.link_totals.at("crsf_telemetry"), 500u) << format_grade(grade);
}

TEST(McapGraderRecordedTest, AugustMassdSessionLostCrsfTelemetryWhileDriving) {
    const auto path = saved_recording(
        "MassD_2026-08-29/auto_battlebot_mrs_buff_mk3_massd_ns_jetson_2026-08-29_09-05-11.mcap");
    if (!std::filesystem::exists(path)) GTEST_SKIP() << "recording not on this machine: " << path;
    const McapGrade grade = grade_mcap(path, PerformanceMonitorConfiguration{});
    EXPECT_TRUE(has_crsf_warning(grade)) << format_grade(grade);
    EXPECT_TRUE(grade.red());
    EXPECT_EQ(grade.link_totals.at("crsf_telemetry"), 0u);
    EXPECT_GT(grade.link_totals.at("transmit"), 1000u) << "the app kept sending throughout";
}

}  // namespace
}  // namespace auto_battlebot
