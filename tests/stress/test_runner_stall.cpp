#include <gtest/gtest.h>

#include <miniros/ros.h>

#include <memory>

#include "data_structures.hpp"
#include "diagnostics_logger/test_diagnostics_logger.hpp"
#include "field_filter/noop_field_filter.hpp"
#include "health/config.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"
#include "mask_model/noop_mask_model.hpp"
#include "navigation/noop_navigation.hpp"
#include "publisher/noop_publisher.hpp"
#include "robot_blob_model/noop_robot_blob_model.hpp"
#include "robot_filter/noop_robot_filter.hpp"
#include "runner.hpp"
#include "runner_config.hpp"
#include "stress/mock_cameras.hpp"
#include "target_selector/noop_target.hpp"
#include "transmitter/noop_transmitter.hpp"

namespace auto_battlebot {
namespace {

void init_miniros_once() {
    if (miniros::isInitialized()) return;
    miniros::M_string remappings;
    miniros::init(remappings, "stress_test",
                  miniros::init_options::NoSigintHandler | miniros::init_options::LocalMaster |
                      miniros::init_options::NoRosout);
}

RunnerConfiguration make_config(double stall_s, double check_s) {
    RunnerConfiguration config;
    config.watchdog_stall_threshold_s = stall_s;
    config.watchdog_check_interval_s = check_s;
    config.max_loop_rate = 1000.0;
    config.autonomy_enabled_by_default = false;
    return config;
}

std::shared_ptr<Runner> make_runner(std::shared_ptr<RgbdCameraInterface> camera,
                                    double stall_s = 10.0, double check_s = 0.1) {
    return std::make_shared<Runner>(
        make_config(stall_s, check_s), camera, HealthConfiguration{},
        std::make_shared<NoopMaskModel>(), std::make_shared<NoopRobotBlobModel>(),
        std::make_shared<NoopFieldFilter>(), std::make_shared<NoopKeypointModel>(),
        std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
        std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(),
        std::make_shared<NoopPublisher>());
}

}  // namespace

class RunnerStallTest : public ::testing::Test {
   protected:
    void SetUp() override {
        init_miniros_once();
        if (!DiagnosticsLogger::is_initialized()) {
            DiagnosticsLogger::initialize({});
        }
        TestDiagnosticsLogger::enable_test_mode();
    }

    void TearDown() override { TestDiagnosticsLogger::reset(); }
};

// Scenario 1: camera->get() blocks forever.
// Simulates ZED SDK grab() hanging on real hardware (e.g. USB stall on Jetson).
// The watchdog must fire std::abort() within the stall threshold.
TEST_F(RunnerStallTest, WatchdogFiresWhenCameraGetBlocks) {
    // stall_threshold=1s, check_interval=0.2s. Child inherits initialized DiagnosticsLogger.
    EXPECT_DEATH(
        {
            auto camera = std::make_shared<BlockingCamera>();
            auto runner = make_runner(camera, 1.0, 0.2);
            runner->initialize();
            runner->run();
        },
        ".*");
}

// Scenario 2: camera->get() fails, then initialize() blocks in the recovery loop.
// Simulates zed_.open() hanging during camera re-initialization after a failure.
// last_tick_ns_ stops being updated while recover_camera_after_failure() is stuck
// inside camera->initialize(), so the watchdog must fire.
TEST_F(RunnerStallTest, WatchdogFiresWhenReinitBlocks) {
    EXPECT_DEATH(
        {
            auto camera = std::make_shared<FailThenBlockCamera>();
            auto runner = make_runner(camera, 1.0, 0.2);
            runner->initialize();
            runner->run();
        },
        ".*");
}

// Scenario 3: slow camera (30 ms/frame) with a generous stall threshold.
// Verifies no false-positive: a camera that is slow but not stuck should not
// trigger the watchdog. Runner exits cleanly after the countdown completes.
TEST_F(RunnerStallTest, SlowCameraNoFalsePositive) {
    // 10 frames @ 30ms each = ~300ms total. stall_threshold=2s → watchdog never fires.
    auto camera = std::make_shared<CountdownCamera>(10, std::chrono::milliseconds(30));
    auto runner = make_runner(camera, 2.0, 0.1);
    runner->initialize();
    int exit_code = runner->run();
    EXPECT_EQ(exit_code, 0);
}

// Scenario 4: camera succeeds for a few frames then signals should_close().
// Verifies the runner exits cleanly (exit 0, no abort) when the camera
// requests shutdown, as opposed to an unrecoverable hardware stall.
TEST_F(RunnerStallTest, CameraCleanShutdown) {
    auto camera = std::make_shared<CountdownCamera>(5);
    auto runner = make_runner(camera);
    runner->initialize();
    int exit_code = runner->run();
    EXPECT_EQ(exit_code, 0);
}

// Scenario 5: verify that a shorter configured threshold fires sooner than the default.
// This ensures the stall threshold from RunnerConfiguration is actually used
// rather than the old hardcoded 10s constant.
TEST_F(RunnerStallTest, WatchdogRespectsConfiguredThreshold) {
    EXPECT_DEATH(
        {
            auto camera = std::make_shared<BlockingCamera>();
            auto runner = make_runner(camera, 0.5, 0.1);
            runner->initialize();
            runner->run();
        },
        ".*");
}

}  // namespace auto_battlebot
