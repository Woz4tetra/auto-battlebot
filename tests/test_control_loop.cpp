// SteppedControlLoop with rate_hz = 0 must run exactly one filter/target/navigation/transmit pass
// per perception frame, matching what a single-threaded pipeline does. SVO replay cannot prove
// that while playback frame selection stays nondeterministic, so prove it here against an inline
// reference sequence written independently of the ControlLoop body.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "control_loop/control_loop.hpp"
#include "control_loop/stepped_control_loop.hpp"
#include "control_loop/threaded_control_loop.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "navigation/navigation_interface.hpp"
#include "robot_descriptions_cache.hpp"
#include "robot_filter/robot_front_back_filter.hpp"
#include "time/clock_interface.hpp"
#include "time/manual_clock.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {
namespace {

// [control_loop] robot_cache_max_age_s default.
constexpr double kCacheMaxAgeS = 0.25;

/** Records every robot set navigation is asked to steer by, and derives a command from it so a
 *  filter difference shows up in the command stream too. */
class RecordingNavigation : public NavigationInterface {
   public:
    bool initialize() override {
        initialize_calls++;
        return true;
    }

    VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                           const TargetSelection &target) override {
        (void)field;
        seen_targets.push_back(target);
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (const auto &robot : robots.descriptions) {
            sum_x += robot.pose.position.x;
            sum_y += robot.pose.position.y;
        }
        seen_robots.push_back(robots);
        VelocityCommand command{sum_x, 0.0, sum_y};
        last_visualization_ = NavigationVisualization{};
        last_visualization_.command = command;
        last_visualization_.robots = std::move(robots);
        return command;
    }

    const NavigationVisualization &get_last_visualization() const override {
        return last_visualization_;
    }

    std::vector<RobotDescriptionsStamped> seen_robots;
    std::vector<TargetSelection> seen_targets;
    int initialize_calls = 0;

   private:
    NavigationVisualization last_visualization_;
};

/** Records commands and reports a fixed stick feedback, so the filter's dead reckoning has an
 *  input. The values are normalized [-1, 1]; consumers scale them by the plant gains. */
class RecordingTransmitter : public TransmitterInterface {
   public:
    bool initialize() override { return true; }

    CommandFeedback update() override {
        update_calls++;
        CommandFeedback feedback;
        feedback.stick_commands[FrameId::OUR_ROBOT_1] = {0.3, 0.0, 0.15};
        return feedback;
    }

    void send(VelocityCommand command) override { sent.push_back(command); }
    bool did_init_button_press() override { return init_button_pressed; }
    void enable() override { enable_calls++; }
    void disable() override { disable_calls++; }
    TransmitterStatus get_status() const override {
        return {.connected = true, .receiving_channels = true};
    }

    std::vector<VelocityCommand> sent;
    int update_calls = 0;
    int enable_calls = 0;
    int disable_calls = 0;
    bool init_button_pressed = false;
};

class SlowInitTransmitter : public RecordingTransmitter {
   public:
    bool initialize() override {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        initialized = true;
        return true;
    }
    void send(VelocityCommand command) override {
        if (!initialized) sent_before_initialize = true;
        RecordingTransmitter::send(command);
    }

    std::atomic<bool> initialized{false};
    std::atomic<bool> sent_before_initialize{false};
};

class RecordingTargetSelector : public TargetSelectorInterface {
   public:
    std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &,
                                              const FieldDescription &,
                                              BehaviorMode mode) override {
        seen_modes.push_back(mode);
        return next;
    }
    std::vector<BehaviorMode> seen_modes;
    std::optional<TargetSelection> next;
};

class OrderRecordingFilter : public RobotFilterInterface {
   public:
    bool initialize(int) override { return true; }
    void predict(double, CommandFeedback) override { calls.push_back("predict"); }
    void correct(ModelResultStamped keypoints, FieldDescription, CameraInfo,
                 ModelResultStamped) override {
        calls.push_back("correct");
        corrected_stamps.push_back(keypoints.header.stamp);
    }
    RobotDescriptionsStamped state() const override { return {}; }
    std::vector<std::string> calls;
    std::vector<double> corrected_stamps;
};

CameraInfo make_camera_info() {
    CameraInfo camera_info;
    camera_info.width = 640;
    camera_info.height = 480;
    camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    camera_info.intrinsics.at<double>(0, 0) = 500.0;
    camera_info.intrinsics.at<double>(1, 1) = 500.0;
    camera_info.intrinsics.at<double>(0, 2) = 320.0;
    camera_info.intrinsics.at<double>(1, 2) = 240.0;
    return camera_info;
}

FieldDescription make_field() {
    FieldDescription field;
    field.child_frame_id = FrameId::FIELD;
    field.size.size.x = 2.0;
    field.size.size.y = 2.0;
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 2) = -1.0;
    tf(2, 3) = 3.0;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

void append_pair(ModelResultStamped &kp, Label label, KeypointLabel front_label,
                 KeypointLabel back_label, int detection_index, double front_x, double back_x,
                 double y, double confidence) {
    Keypoint front;
    front.label = label;
    front.keypoint_label = front_label;
    front.x = front_x;
    front.y = y;
    front.confidence = confidence;
    front.detection_index = detection_index;
    kp.keypoints.push_back(front);

    Keypoint back = front;
    back.keypoint_label = back_label;
    back.x = back_x;
    back.confidence = confidence;
    kp.keypoints.push_back(back);
}

RobotFrontBackFilterConfiguration make_filter_config() {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT, KeypointLabel::MRS_BUFF_MK3_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK, KeypointLabel::MRS_BUFF_MK3_BACK};
    config.label_to_frame_ids = {{Label::MRS_BUFF_MK3, {FrameId::OUR_ROBOT_1}},
                                 {Label::OPPONENT, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::OUR_ROBOT_1;
    config.max_jump_distance = 1.0;
    config.max_consecutive_jump_rejects = 1;
    config.blob_overwrite_min_distance_meters = 0.3;
    config.blob_overwrite_size_scale = 0.5;
    config.our_robot_hold_window_s = 0.4;
    config.robot_keypoint_tracker_config.min_length_meters = 0.05;
    config.robot_keypoint_tracker_config.max_length_meters = 2.0;
    config.robot_keypoint_tracker_config.min_confidence = 0.2;
    config.robot_keypoint_tracker_config.max_candidates = 8;
    return config;
}

/** Frames covering measured tracks, our-robot dropouts, and opponent dropouts. */
std::vector<ControlMeasurement> make_measurements(int count) {
    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    std::vector<ControlMeasurement> out;
    for (int i = 0; i < count; ++i) {
        ControlMeasurement m;
        m.camera_info = camera_info;
        m.field_description = field;
        m.keypoints.header.frame_id = FrameId::CAMERA;
        m.keypoints.header.stamp = 1.0 + 0.0333 * i;
        m.robot_blob_keypoints.header.frame_id = FrameId::CAMERA;
        m.robot_blob_keypoints.header.stamp = m.keypoints.header.stamp;

        if ((i % 40) < 35) {
            const double x = 260.0 + 40.0 * std::sin(i * 0.05);
            append_pair(m.keypoints, Label::MRS_BUFF_MK3, KeypointLabel::MRS_BUFF_MK3_FRONT,
                        KeypointLabel::MRS_BUFF_MK3_BACK, 1, x, x + 40.0, 220.0, 0.9);
        }
        if ((i % 25) < 22) {
            const double x = 400.0 - 30.0 * std::cos(i * 0.07);
            append_pair(m.keypoints, Label::OPPONENT, KeypointLabel::OPPONENT_FRONT,
                        KeypointLabel::OPPONENT_BACK, 2, x, x + 35.0, 300.0, 0.85);
        }
        out.push_back(std::move(m));
    }
    return out;
}

void expect_robots_equal(const RobotDescriptionsStamped &a, const RobotDescriptionsStamped &b,
                         int index) {
    ASSERT_EQ(a.descriptions.size(), b.descriptions.size()) << "frame " << index;
    for (size_t i = 0; i < a.descriptions.size(); ++i) {
        const std::string where = "frame " + std::to_string(index) + " desc " + std::to_string(i);
        EXPECT_EQ(a.descriptions[i].frame_id, b.descriptions[i].frame_id) << where;
        EXPECT_EQ(a.descriptions[i].is_stale, b.descriptions[i].is_stale) << where;
        EXPECT_EQ(a.descriptions[i].pose.position.x, b.descriptions[i].pose.position.x) << where;
        EXPECT_EQ(a.descriptions[i].pose.position.y, b.descriptions[i].pose.position.y) << where;
    }
}

}  // namespace

TEST(ControlLoopTest, SteppedZeroRateMatchesInlineSequence) {
    const auto measurements = make_measurements(200);

    std::vector<VelocityCommand> reference_commands;
    std::vector<RobotDescriptionsStamped> reference_robots;
    {
        auto config = make_filter_config();
        RobotFrontBackFilter filter(config);
        ASSERT_TRUE(filter.initialize(1));
        RecordingNavigation navigation;
        RecordingTransmitter transmitter;
        RobotDescriptionsCache cache(kCacheMaxAgeS);
        auto clock = std::make_shared<ManualClock>();

        for (const auto &m : measurements) {
            const CommandFeedback feedback = transmitter.update();
            clock->set(m.keypoints.header.stamp);
            filter.predict(clock->now(), feedback);
            filter.correct(m.keypoints, m.field_description, m.camera_info, m.robot_blob_keypoints);
            const RobotDescriptionsStamped robots = filter.state();
            auto cached = cache.resolve(robots);
            transmitter.send(navigation.update(cached.robots, m.field_description, {}));
            reference_robots.push_back(robots);
        }
        reference_commands = transmitter.sent;
    }

    std::vector<VelocityCommand> loop_commands;
    std::vector<RobotDescriptionsStamped> loop_robots;
    {
        auto config = make_filter_config();
        auto filter = std::make_shared<RobotFrontBackFilter>(config);
        ASSERT_TRUE(filter->initialize(1));
        auto navigation = std::make_shared<RecordingNavigation>();
        auto transmitter = std::make_shared<RecordingTransmitter>();
        auto clock = std::make_shared<ManualClock>();
        auto loop = std::make_shared<ControlLoop>(filter, nullptr, navigation, transmitter, clock,
                                                  nullptr, kCacheMaxAgeS);
        SteppedControlLoop driver(loop, 0.0);
        driver.start();

        for (const auto &m : measurements) {
            driver.pump_input();
            clock->set(m.keypoints.header.stamp);
            loop->submit_measurement(m);
            driver.advance_to(m.keypoints.header.stamp);
            loop_robots.push_back(loop->latest_output().robots);
        }
        loop_commands = transmitter->sent;
    }

    ASSERT_EQ(reference_commands.size(), measurements.size());
    ASSERT_EQ(loop_commands.size(), reference_commands.size());
    ASSERT_EQ(loop_robots.size(), reference_robots.size());

    for (size_t i = 0; i < reference_commands.size(); ++i) {
        const int index = static_cast<int>(i);
        ASSERT_NO_FATAL_FAILURE(expect_robots_equal(reference_robots[i], loop_robots[i], index));
        EXPECT_EQ(reference_commands[i].forward_mps, loop_commands[i].forward_mps)
            << "frame " << index;
        EXPECT_EQ(reference_commands[i].yaw_rate_radps, loop_commands[i].yaw_rate_radps)
            << "frame " << index;
    }

    // Guard against a scenario that passes by producing nothing.
    const bool any_motion =
        std::any_of(loop_commands.begin(), loop_commands.end(),
                    [](const VelocityCommand &c) { return c.forward_mps != 0.0; });
    EXPECT_TRUE(any_motion);
}

TEST(ControlLoopTest, ZeroRateRunsExactlyOneCyclePerAdvance) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, nullptr, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);

    const auto measurements = make_measurements(10);
    for (const auto &m : measurements) {
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        // Deliberately uneven advance times: at rate_hz = 0 the cycle count must not depend on
        // them, which is why 0 exists rather than setting rate_hz to the camera rate.
        driver.advance_to(m.keypoints.header.stamp * 1.5);
    }
    EXPECT_EQ(transmitter->sent.size(), measurements.size());
}

class CountingClock : public ClockInterface {
   public:
    double now() const override {
        ++reads;
        return 0.0;
    }
    mutable int reads = 0;
};

TEST(ControlLoopTest, NoCycleReadsClockBeforeFirstFrame) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto clock = std::make_shared<CountingClock>();
    auto loop = std::make_shared<ControlLoop>(filter, nullptr, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);

    for (int i = 0; i < 5; ++i) driver.advance_to(1.0 + i);

    EXPECT_EQ(clock->reads, 0);
    EXPECT_TRUE(transmitter->sent.empty());
}

TEST(ControlLoopTest, NonZeroRateRunsMultipleCyclesPerFrame) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, nullptr, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);
    // 240 Hz against 30 Hz frames: 8 cycles per frame after the first.
    SteppedControlLoop driver(loop, 240.0);

    const auto measurements = make_measurements(10);
    for (const auto &m : measurements) {
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        driver.advance_to(m.keypoints.header.stamp);
    }
    // 240 Hz over a 33.3 ms frame is 7.99 periods, so floor() gives 7 or 8 depending on
    // accumulated rounding. Assert the rate is tracked, not an exact count that would make the
    // test a hostage to floating point.
    const size_t cycles = transmitter->sent.size();
    const size_t frames_after_first = measurements.size() - 1;
    EXPECT_GE(cycles, 1u + 7u * frames_after_first);
    EXPECT_LE(cycles, 1u + 8u * frames_after_first);
}

TEST(ControlLoopTest, MeasurementIsCorrectedExactlyOnce) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, nullptr, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);

    const auto measurements = make_measurements(2);
    clock->set(measurements[0].keypoints.header.stamp);
    loop->submit_measurement(measurements[0]);
    driver.advance_to(measurements[0].keypoints.header.stamp);
    const auto after_first = loop->latest_output().robots;

    driver.advance_to(measurements[0].keypoints.header.stamp);
    driver.advance_to(measurements[0].keypoints.header.stamp);
    const auto after_extra_cycles = loop->latest_output().robots;

    ASSERT_EQ(after_first.descriptions.size(), after_extra_cycles.descriptions.size());
    for (size_t i = 0; i < after_first.descriptions.size(); ++i) {
        // Every track is stale now, so a cycle without a measurement must leave poses untouched:
        // this filter only propagates inside correct().
        EXPECT_EQ(after_first.descriptions[i].pose.position.x,
                  after_extra_cycles.descriptions[i].pose.position.x);
    }
}

TEST(ControlLoopTest, BehaviorModeReachesSelectorAndOutput) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto selector = std::make_shared<RecordingTargetSelector>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, selector, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);
    loop->set_behavior_mode(BehaviorMode::RUN_AWAY);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();

    const auto measurements = make_measurements(3);
    for (const auto &m : measurements) {
        driver.pump_input();
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        driver.advance_to(m.keypoints.header.stamp);
    }

    ASSERT_FALSE(selector->seen_modes.empty());
    for (const auto mode : selector->seen_modes) {
        EXPECT_EQ(mode, BehaviorMode::RUN_AWAY);
    }
    EXPECT_EQ(loop->latest_output().behavior_mode, BehaviorMode::RUN_AWAY);
}

TEST(ControlLoopTest, BehaviorModeIsLoggedEveryCycle) {
    auto runner_logger = DiagnosticsLogger::get_logger("runner");
    runner_logger->clear();

    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto selector = std::make_shared<RecordingTargetSelector>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, selector, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);

    loop->set_behavior_mode(BehaviorMode::RUN_AWAY);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();

    const auto measurements = make_measurements(1);
    for (const auto &m : measurements) {
        driver.pump_input();
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        driver.advance_to(m.keypoints.header.stamp);
    }

    bool found = false;
    for (const auto &snapshot : runner_logger->get_snapshots()) {
        if (snapshot.subsection != "navigation") continue;
        auto it = snapshot.values.find("behavior_mode");
        if (it == snapshot.values.end()) continue;
        found = true;
        EXPECT_EQ(diagnostic_scalar_to_string(it->second), "RUN_AWAY");
    }
    EXPECT_TRUE(found) << "no behavior_mode value in the runner/navigation diagnostics";
    runner_logger->clear();
}

TEST(ControlLoopTest, TargetOverrideBypassesSelectorUntilCleared) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto selector = std::make_shared<RecordingTargetSelector>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, selector, navigation, transmitter, clock,
                                              nullptr, kCacheMaxAgeS);
    loop->set_behavior_mode(BehaviorMode::RUN_AWAY);
    loop->set_target_override(Pose2D{.x = 0.4, .y = -0.3, .yaw = 0.0});
    SteppedControlLoop driver(loop, 0.0);
    driver.start();

    const auto measurements = make_measurements(2);
    auto step = [&](const ControlMeasurement &m) {
        driver.pump_input();
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        driver.advance_to(m.keypoints.header.stamp);
    };

    step(measurements[0]);
    EXPECT_TRUE(selector->seen_modes.empty());
    const ControlOutput held = loop->latest_output();
    EXPECT_TRUE(held.manual_target);
    EXPECT_DOUBLE_EQ(held.target.pose.x, 0.4);
    EXPECT_DOUBLE_EQ(held.target.pose.y, -0.3);
    EXPECT_EQ(held.target.label, Label::EMPTY);
    EXPECT_EQ(held.target.mode, BehaviorMode::RUN_AWAY);

    loop->set_target_override(std::nullopt);
    step(measurements[1]);
    EXPECT_FALSE(selector->seen_modes.empty());
    EXPECT_FALSE(loop->latest_output().manual_target);
}

TEST(ControlLoopTest, HeldTargetResumesAfterAnOverrideClears) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto selector = std::make_shared<RecordingTargetSelector>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, selector, navigation,
                                              std::make_shared<RecordingTransmitter>(), clock,
                                              nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();
    const auto measurements = make_measurements(3);
    auto step = [&](const ControlMeasurement &m) {
        driver.pump_input();
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        driver.advance_to(m.keypoints.header.stamp);
    };

    selector->next = TargetSelection{.pose = Pose2D{.x = 0.5, .y = 0.1, .yaw = 0.0}};
    step(measurements[0]);
    loop->set_target_override(Pose2D{.x = -0.4, .y = 0.0, .yaw = 0.0});
    step(measurements[1]);
    loop->set_target_override(std::nullopt);
    selector->next = std::nullopt;
    step(measurements[2]);

    ASSERT_FALSE(navigation->seen_targets.empty());
    EXPECT_DOUBLE_EQ(navigation->seen_targets.back().pose.x, 0.5);
}

TEST(ControlLoopTest, ReusedTargetCarriesTheCurrentBehaviorMode) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto navigation = std::make_shared<RecordingNavigation>();
    auto selector = std::make_shared<RecordingTargetSelector>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(filter, selector, navigation,
                                              std::make_shared<RecordingTransmitter>(), clock,
                                              nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();
    const auto measurements = make_measurements(2);
    auto step = [&](const ControlMeasurement &m) {
        driver.pump_input();
        clock->set(m.keypoints.header.stamp);
        loop->submit_measurement(m);
        driver.advance_to(m.keypoints.header.stamp);
    };

    selector->next = TargetSelection{.pose = Pose2D{.x = 0.5, .y = 0.1, .yaw = 0.0}};
    step(measurements[0]);
    selector->next = std::nullopt;
    loop->set_behavior_mode(BehaviorMode::RUN_AWAY);
    step(measurements[1]);

    ASSERT_FALSE(navigation->seen_targets.empty());
    EXPECT_DOUBLE_EQ(navigation->seen_targets.back().pose.x, 0.5);
    EXPECT_EQ(navigation->seen_targets.back().mode, BehaviorMode::RUN_AWAY);
}

TEST(ControlLoopTest, WatchdogStampAdvancesWhileWaitingForTheField) {
    auto config = make_filter_config();
    auto filter = std::make_shared<RobotFrontBackFilter>(config);
    ASSERT_TRUE(filter->initialize(1));
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(
        filter, nullptr, std::make_shared<RecordingNavigation>(),
        std::make_shared<RecordingTransmitter>(), clock, nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();
    const auto before = loop->last_cycle_wall_time();
    clock->set(1.0);
    driver.advance_to(1.0);
    EXPECT_GT(loop->last_cycle_wall_time(), before);
}

TEST(ControlLoopTest, SubmitMeasurementReplacesUnconsumed) {
    auto filter = std::make_shared<OrderRecordingFilter>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(
        filter, nullptr, std::make_shared<RecordingNavigation>(),
        std::make_shared<RecordingTransmitter>(), clock, nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();
    const auto measurements = make_measurements(2);
    loop->submit_measurement(measurements[0]);
    loop->submit_measurement(measurements[1]);
    driver.advance_to(1.0);
    driver.advance_to(1.0);
    ASSERT_EQ(filter->corrected_stamps.size(), 1u);
    EXPECT_EQ(filter->corrected_stamps[0], measurements[1].keypoints.header.stamp);
}

TEST(ControlLoopTest, InitialAutonomyStateReachesTransmitter) {
    auto make_loop = [](std::shared_ptr<RecordingTransmitter> transmitter) {
        return std::make_shared<ControlLoop>(
            std::make_shared<OrderRecordingFilter>(), nullptr,
            std::make_shared<RecordingNavigation>(), std::move(transmitter),
            std::make_shared<ManualClock>(), nullptr, kCacheMaxAgeS);
    };

    auto enabled_transmitter = std::make_shared<RecordingTransmitter>();
    auto enabled_loop = make_loop(enabled_transmitter);
    enabled_loop->run_cycle();
    EXPECT_EQ(enabled_transmitter->enable_calls, 1);
    EXPECT_EQ(enabled_transmitter->disable_calls, 0);
    enabled_loop->run_cycle();
    EXPECT_EQ(enabled_transmitter->enable_calls, 1);

    auto disabled_transmitter = std::make_shared<RecordingTransmitter>();
    auto disabled_loop = make_loop(disabled_transmitter);
    disabled_loop->set_autonomy_enabled(false);
    disabled_loop->run_cycle();
    EXPECT_EQ(disabled_transmitter->enable_calls, 0);
    EXPECT_EQ(disabled_transmitter->disable_calls, 1);
}

TEST(ControlLoopTest, SteppedPumpInputLatchesInitButtonWithoutCycling) {
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<OrderRecordingFilter>(), nullptr, std::make_shared<RecordingNavigation>(),
        transmitter, std::make_shared<ManualClock>(), nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);
    transmitter->init_button_pressed = true;
    driver.pump_input();
    EXPECT_EQ(transmitter->update_calls, 1);
    EXPECT_TRUE(transmitter->sent.empty());
    EXPECT_TRUE(driver.take_init_button_press());
    EXPECT_FALSE(driver.take_init_button_press());
}

class TimestampingTransmitter : public RecordingTransmitter {
   public:
    void send(VelocityCommand command) override {
        std::lock_guard<std::mutex> lock(mutex);
        send_times.push_back(std::chrono::steady_clock::now());
        if (send_times.size() == 1) std::this_thread::sleep_for(std::chrono::milliseconds(40));
        RecordingTransmitter::send(command);
    }

    std::mutex mutex;
    std::vector<std::chrono::steady_clock::time_point> send_times;
};

TEST(ControlLoopTest, SteppedCatchUpCappedAfterStall) {
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<OrderRecordingFilter>(), nullptr, std::make_shared<RecordingNavigation>(),
        transmitter, std::make_shared<ManualClock>(), nullptr, kCacheMaxAgeS);
    constexpr int kCap = 5;
    SteppedControlLoop driver(loop, 240.0, kCap);

    loop->submit_measurement(make_measurements(1)[0]);
    driver.advance_to(0.0);
    ASSERT_EQ(transmitter->sent.size(), 1u);
    driver.advance_to(10.0);
    EXPECT_EQ(transmitter->sent.size(), 1u + kCap) << "a 10 s stall replayed more than the cap";
}

TEST(ControlLoopTest, ThreadedResyncsAfterOverrun) {
    auto transmitter = std::make_shared<TimestampingTransmitter>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<OrderRecordingFilter>(), nullptr, std::make_shared<RecordingNavigation>(),
        transmitter, std::make_shared<ManualClock>(), nullptr, kCacheMaxAgeS);
    ThreadedControlLoop driver(loop, 250.0, 20.0);
    loop->submit_measurement(make_measurements(1)[0]);
    ASSERT_TRUE(driver.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    driver.stop();

    std::lock_guard<std::mutex> lock(transmitter->mutex);
    ASSERT_GE(transmitter->send_times.size(), 6u);
    for (size_t i = 2; i < 6; ++i) {
        const auto gap = transmitter->send_times[i] - transmitter->send_times[i - 1];
        EXPECT_GE(gap, std::chrono::milliseconds(2)) << "cycle " << i << " ran back to back";
    }
}

TEST(ControlLoopTest, ThreadedPumpInputLeavesTheTransmitterToTheControlThread) {
    auto transmitter = std::make_shared<RecordingTransmitter>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<OrderRecordingFilter>(), nullptr, std::make_shared<RecordingNavigation>(),
        transmitter, std::make_shared<ManualClock>(), nullptr, kCacheMaxAgeS);
    ThreadedControlLoop driver(loop, 250.0, 20.0);
    driver.pump_input();
    EXPECT_EQ(transmitter->update_calls, 0);
}

TEST(ControlLoopTest, ThreadedStartInitializesTransmitterBeforeFirstCycle) {
    auto transmitter = std::make_shared<SlowInitTransmitter>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<OrderRecordingFilter>(), nullptr, std::make_shared<RecordingNavigation>(),
        transmitter, std::make_shared<ManualClock>(), nullptr, kCacheMaxAgeS);
    ThreadedControlLoop driver(loop, 250.0, 20.0);
    loop->submit_measurement(make_measurements(1)[0]);
    EXPECT_TRUE(driver.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    driver.stop();
    EXPECT_TRUE(transmitter->initialized);
    EXPECT_FALSE(transmitter->sent.empty());
    EXPECT_FALSE(transmitter->sent_before_initialize);
}

TEST(ControlLoopTest, FilterCorrectsBeforeItPredicts) {
    auto filter = std::make_shared<OrderRecordingFilter>();
    auto clock = std::make_shared<ManualClock>();
    auto loop = std::make_shared<ControlLoop>(
        filter, nullptr, std::make_shared<RecordingNavigation>(),
        std::make_shared<RecordingTransmitter>(), clock, nullptr, kCacheMaxAgeS);
    SteppedControlLoop driver(loop, 0.0);
    driver.start();
    const auto measurements = make_measurements(1);
    clock->set(measurements[0].keypoints.header.stamp);
    loop->submit_measurement(measurements[0]);
    driver.advance_to(measurements[0].keypoints.header.stamp);
    ASSERT_GE(filter->calls.size(), 2u);
    EXPECT_EQ(filter->calls[0], "correct");
    EXPECT_EQ(filter->calls[1], "predict");
}

}  // namespace auto_battlebot
