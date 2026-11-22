#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

#include "health/performance_monitor.hpp"

namespace auto_battlebot {
namespace {

constexpr double kTick = 0.02;

std::optional<PerformanceWarning> find_active(const PerformanceMonitor &monitor,
                                              PerformanceCheck check) {
    for (const auto &warning : monitor.active()) {
        if (warning.check == check) return warning;
    }
    return std::nullopt;
}

std::optional<PerformanceWarning> find_ended(const PerformanceMonitor &monitor,
                                             PerformanceCheck check) {
    for (const auto &warning : monitor.ended()) {
        if (warning.check == check) return warning;
    }
    return std::nullopt;
}

MonitorTick driving_tick(double t) {
    MonitorTick tick;
    tick.t = t;
    tick.autonomy_driving = true;
    tick.field = FieldBox{.half_x = 0.75, .half_y = 0.75};
    tick.our_pose = Pose2D{0.0, 0.0, 0.0};
    tick.opponent_expected = true;
    tick.opponent_present = true;
    return tick;
}

template <typename F>
double run_ticks(PerformanceMonitor &monitor, double start, double seconds, F &&make_tick) {
    double t = start;
    const int steps = static_cast<int>(std::lround(seconds / kTick));
    for (int i = 0; i < steps; ++i) {
        t += kTick;
        monitor.record_tick(make_tick(t, i));
        monitor.evaluate(t);
    }
    return t;
}

TEST(PerformanceMonitorLinkTest, SilentConfiguredLinkGoesRedOnlyAfterTheGracePeriod) {
    PerformanceMonitorConfiguration config;
    PerformanceMonitor monitor(config);
    monitor.set_link_expected(MonitorLink::ESPNOW, true, 0.0);
    ASSERT_GT(config.link_grace_s, config.espnow_max_age_s);

    monitor.evaluate(config.link_grace_s - 0.1);
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::ESPNOW_LINK));

    monitor.evaluate(config.link_grace_s + 0.1);
    const auto warning = find_active(monitor, PerformanceCheck::ESPNOW_LINK);
    ASSERT_TRUE(warning);
    EXPECT_EQ(warning->severity, WarningSeverity::RED);
    EXPECT_GT(warning->values.at("age_s"), config.espnow_max_age_s);
}

TEST(PerformanceMonitorLinkTest, CrsfTelemetryGoesRedPastItsMaxAge) {
    PerformanceMonitorConfiguration config;
    PerformanceMonitor monitor(config);
    monitor.set_link_expected(MonitorLink::CRSF_TELEMETRY, true, 0.0);
    monitor.evaluate(config.crsf_telemetry_max_age_s - 0.1);
    EXPECT_TRUE(monitor.active().empty()) << "a zero rate waits for the silence check";
    monitor.evaluate(config.crsf_telemetry_max_age_s + 0.1);
    const auto warning = find_active(monitor, PerformanceCheck::CRSF_TELEMETRY_LINK);
    ASSERT_TRUE(warning);
    EXPECT_EQ(warning->severity, WarningSeverity::RED);
}

TEST(PerformanceMonitorLinkTest, SeverityChangeStartsANewWarning) {
    PerformanceMonitorConfiguration config;
    PerformanceMonitor monitor(config);
    monitor.set_link_expected(MonitorLink::CAMERA_FRAMES, true, 0.0);
    monitor.evaluate(4.0);
    ASSERT_EQ(find_active(monitor, PerformanceCheck::CAMERA_FRAMES_LINK)->severity,
              WarningSeverity::RED);
    for (double t = 4.0; t < 9.0; t += 0.01) {
        if (std::fmod(t, 0.2) < 0.01) monitor.add_link_events(MonitorLink::CAMERA_FRAMES);
        monitor.evaluate(t);
    }
    const auto slow = find_active(monitor, PerformanceCheck::CAMERA_FRAMES_LINK);
    ASSERT_TRUE(slow);
    EXPECT_EQ(slow->severity, WarningSeverity::WARN);
    const auto silent = find_ended(monitor, PerformanceCheck::CAMERA_FRAMES_LINK);
    ASSERT_TRUE(silent);
    EXPECT_EQ(silent->severity, WarningSeverity::RED);
    EXPECT_LE(*silent->end_s, slow->start_s);
}

TEST(PerformanceMonitorLinkTest, LinkThatComesBackEndsTheWarningWithItsTimes) {
    PerformanceMonitorConfiguration config;
    PerformanceMonitor monitor(config);
    monitor.set_link_expected(MonitorLink::ESPNOW, true, 0.0);
    monitor.evaluate(4.0);
    ASSERT_TRUE(find_active(monitor, PerformanceCheck::ESPNOW_LINK));

    uint64_t total = 0;
    for (double t = 4.0; t < 5.0; t += 0.01) {
        monitor.set_link_total(MonitorLink::ESPNOW, ++total);
        monitor.evaluate(t);
    }
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::ESPNOW_LINK));
    const auto ended = find_ended(monitor, PerformanceCheck::ESPNOW_LINK);
    ASSERT_TRUE(ended);
    EXPECT_DOUBLE_EQ(ended->start_s, 4.0);
    ASSERT_TRUE(ended->end_s);
    EXPECT_LT(*ended->end_s, 4.05);
}

TEST(PerformanceMonitorLinkTest, SlowButAliveLinkWarnsInsteadOfGoingRed) {
    PerformanceMonitorConfiguration config;
    PerformanceMonitor monitor(config);
    monitor.set_link_expected(MonitorLink::CAMERA_FRAMES, true, 0.0);
    for (double t = 0.0; t < 8.0; t += 0.01) {
        if (std::fmod(t, 0.2) < 0.01) monitor.add_link_events(MonitorLink::CAMERA_FRAMES);
        monitor.evaluate(t);
    }
    const auto warning = find_active(monitor, PerformanceCheck::CAMERA_FRAMES_LINK);
    ASSERT_TRUE(warning);
    EXPECT_EQ(warning->severity, WarningSeverity::WARN);
    EXPECT_NEAR(warning->values.at("rate_hz"), 5.0, 0.5);
}

TEST(PerformanceMonitorLinkTest, UnconfiguredLinkNeverWarns) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    monitor.evaluate(30.0);
    EXPECT_TRUE(monitor.active().empty());
}

TEST(PerformanceMonitorLinkTest, PolledSourceFeedsTheLink) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    uint64_t sent = 0;
    monitor.set_link_source(MonitorLink::RELAY_PUBLISH, [&sent] { return sent; });
    ASSERT_TRUE(monitor.has_link_source(MonitorLink::RELAY_PUBLISH));
    monitor.set_link_expected(MonitorLink::RELAY_PUBLISH, true, 0.0);
    for (double t = 0.0; t < 6.0; t += 0.05) {
        ++sent;
        monitor.evaluate(t);
    }
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::RELAY_PUBLISH_LINK));
    monitor.evaluate(8.0);
    EXPECT_TRUE(find_active(monitor, PerformanceCheck::RELAY_PUBLISH_LINK));
}

TEST(PerformanceMonitorCheckTest, CalibrationFiresWhenOurRobotSitsOutsideTheFieldBox) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int i) {
        MonitorTick tick = driving_tick(t);
        if (i % 12 == 0) tick.our_pose = Pose2D{0.9, 0.1, 0.0};
        return tick;
    });
    const auto warning = find_active(monitor, PerformanceCheck::CALIBRATION);
    ASSERT_TRUE(warning);
    EXPECT_NEAR(warning->values.at("fraction"), 1.0 / 12.0, 0.01);
}

TEST(PerformanceMonitorCheckTest, CalibrationStaysQuietBelowTheFraction) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int i) {
        MonitorTick tick = driving_tick(t);
        if (i % 100 == 0) tick.our_pose = Pose2D{0.9, 0.1, 0.0};
        return tick;
    });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::CALIBRATION));
}

TEST(PerformanceMonitorCheckTest, PhantomTrackFiresOnALargeSigmaThatDoesNotMove) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 2.0, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.tracks.push_back({.name = "neutral_robot_1", .sigma_m = 122.0, .speed_mps = 0.0});
        tick.tracks.push_back({.name = "their_robot_1", .sigma_m = 0.05, .speed_mps = 0.0});
        return tick;
    });
    const auto warning = find_active(monitor, PerformanceCheck::PHANTOM_TRACK);
    ASSERT_TRUE(warning);
    EXPECT_NE(warning->message.find("neutral_robot_1"), std::string::npos);
    EXPECT_DOUBLE_EQ(warning->values.at("sigma_m"), 122.0);
}

TEST(PerformanceMonitorCheckTest, MovingOrTightTracksAreNotPhantoms) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 3.0, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.tracks.push_back({.name = "their_robot_1", .sigma_m = 3.0, .speed_mps = 0.5});
        tick.tracks.push_back({.name = "their_robot_2", .sigma_m = 0.2, .speed_mps = 0.0});
        return tick;
    });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::PHANTOM_TRACK));
}

TEST(PerformanceMonitorCheckTest, PhantomNeedsTheMinimumDuration) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 0.5, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.tracks.push_back({.name = "neutral_robot_1", .sigma_m = 122.0, .speed_mps = 0.0});
        return tick;
    });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::PHANTOM_TRACK));
}

TEST(PerformanceMonitorCheckTest, SlewBindingFiresWhenTheSentCommandLags) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int i) {
        MonitorTick tick = driving_tick(t);
        tick.policy_linear = -0.6;
        tick.sent_linear = i % 4 == 0 ? -0.33 : -0.6;
        return tick;
    });
    const auto warning = find_active(monitor, PerformanceCheck::SLEW_BINDING);
    ASSERT_TRUE(warning);
    EXPECT_NEAR(warning->values.at("fraction"), 0.25, 0.01);
}

TEST(PerformanceMonitorCheckTest, SlewIsNotJudgedWhileAutonomyIsOff) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.autonomy_driving = false;
        tick.policy_linear = -0.6;
        tick.sent_linear = 0.0;
        return tick;
    });
    EXPECT_TRUE(monitor.active().empty());
}

TEST(PerformanceMonitorCheckTest, GuardBindingFiresBelowTheScaleFloor) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int i) {
        MonitorTick tick = driving_tick(t);
        tick.guard_scale = i % 10 == 0 ? 0.48 : 1.0;
        return tick;
    });
    EXPECT_TRUE(find_active(monitor, PerformanceCheck::GUARD_BINDING));
}

TEST(PerformanceMonitorCheckTest, ThermalTripIsRed) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 0.1, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.guard_state = kGuardStateTripped;
        return tick;
    });
    const auto warning = find_active(monitor, PerformanceCheck::THERMAL_TRIP);
    ASSERT_TRUE(warning);
    EXPECT_EQ(warning->severity, WarningSeverity::RED);
}

TEST(PerformanceMonitorCheckTest, FirmwareProtocolMismatchIsRedAndNamesTheBuild) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 0.1, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.firmware = FirmwareInfo{.protocol_version = 2, .build = "abc12345"};
        tick.host_protocol_version = 1;
        return tick;
    });
    const auto warning = find_active(monitor, PerformanceCheck::FIRMWARE_PROTOCOL);
    ASSERT_TRUE(warning);
    EXPECT_EQ(warning->severity, WarningSeverity::RED);
    EXPECT_NE(warning->message.find("abc12345"), std::string::npos);
}

TEST(PerformanceMonitorCheckTest, MatchingFirmwareProtocolIsQuiet) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 0.1, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.firmware = FirmwareInfo{.protocol_version = 1, .build = "abc12345"};
        tick.host_protocol_version = 1;
        return tick;
    });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::FIRMWARE_PROTOCOL));
}

TEST(PerformanceMonitorCheckTest, TrackingCoverageBelowTheFloorWarns) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int i) {
        MonitorTick tick = driving_tick(t);
        if (i % 5 == 0) tick.our_pose.reset();
        if (i % 4 == 0) tick.opponent_present = false;
        return tick;
    });
    EXPECT_TRUE(find_active(monitor, PerformanceCheck::OUR_TRACKING));
    EXPECT_TRUE(find_active(monitor, PerformanceCheck::OPPONENT_TRACKING));
}

TEST(PerformanceMonitorCheckTest, NoOpponentExpectedMeansNoOpponentCoverageCheck) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.opponent_expected = false;
        tick.opponent_present = false;
        return tick;
    });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::OPPONENT_TRACKING));
}

TEST(PerformanceMonitorCheckTest, EspNowAckAndCommandAgeWarn) {
    PerformanceMonitor ack_monitor(PerformanceMonitorConfiguration{});
    run_ticks(ack_monitor, 0.0, 2.0, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.espnow_ack_percent = 60.0;
        return tick;
    });
    const auto ack = find_active(ack_monitor, PerformanceCheck::ESPNOW_LINK);
    ASSERT_TRUE(ack);
    EXPECT_DOUBLE_EQ(ack->values.at("ack_percent"), 60.0);

    PerformanceMonitor age_monitor(PerformanceMonitorConfiguration{});
    run_ticks(age_monitor, 0.0, 2.0, [](double t, int) {
        MonitorTick tick = driving_tick(t);
        tick.command_age_ms = 400.0;
        return tick;
    });
    EXPECT_TRUE(find_active(age_monitor, PerformanceCheck::ESPNOW_LINK));
}

MonitorTick turning_tick(double t, double imu_sign) {
    MonitorTick tick = driving_tick(t);
    const double rate = 3.0 * std::sin(2.0 * t);
    const double heading = -1.5 * std::cos(2.0 * t);
    tick.our_pose = Pose2D{0.0, 0.0, std::remainder(heading, 2.0 * std::numbers::pi)};
    tick.our_heading_measured = true;
    tick.imu_yaw_rate_ccw_radps = imu_sign * rate;
    return tick;
}

TEST(PerformanceMonitorCheckTest, ImuThatAgreesWithTheTrackedHeadingIsQuiet) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int) { return turning_tick(t, 1.0); });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::YAW_RATE_DISAGREEMENT));
}

TEST(PerformanceMonitorCheckTest, ImuWithTheWrongSignDisagrees) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int) { return turning_tick(t, -1.0); });
    const auto warning = find_active(monitor, PerformanceCheck::YAW_RATE_DISAGREEMENT);
    ASSERT_TRUE(warning);
    EXPECT_LT(warning->values.at("correlation"), -0.9);
}

TEST(PerformanceMonitorCheckTest, StationaryRobotDoesNotJudgeYawRate) {
    PerformanceMonitor monitor(PerformanceMonitorConfiguration{});
    run_ticks(monitor, 0.0, 5.0, [](double t, int i) {
        MonitorTick tick = driving_tick(t);
        tick.our_heading_measured = true;
        tick.imu_yaw_rate_ccw_radps = i % 2 == 0 ? 0.01 : -0.01;
        return tick;
    });
    EXPECT_FALSE(find_active(monitor, PerformanceCheck::YAW_RATE_DISAGREEMENT));
}

TEST(PerformanceMonitorCheckTest, CheckNamesAreLowerCase) {
    EXPECT_EQ(check_name(PerformanceCheck::CRSF_TELEMETRY_LINK), "crsf_telemetry_link");
    EXPECT_EQ(link_check(MonitorLink::IMU), PerformanceCheck::IMU_LINK);
}

}  // namespace
}  // namespace auto_battlebot
