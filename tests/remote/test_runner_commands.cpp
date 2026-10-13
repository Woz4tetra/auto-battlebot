#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "control_loop/stepped_control_loop.hpp"
#include "field_filter/noop_field_filter.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"
#include "mask_model/noop_mask_model.hpp"
#include "navigation/noop_navigation.hpp"
#include "publisher/noop_publisher.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"
#include "robot_blob_model/noop_robot_blob_model.hpp"
#include "robot_filter/noop_robot_filter.hpp"
#include "runner.hpp"
#include "target_selector/noop_target.hpp"
#include "transmitter/noop_transmitter.hpp"

namespace auto_battlebot {
namespace {

// Toggled on the runner's recording worker and read by the tick, hence the atomics.
class RecordingCamera : public NoopRgbdCamera {
   public:
    bool set_recording_enabled(bool enabled) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(toggle_delay_ms.load()));
        recording = enabled;
        std::lock_guard<std::mutex> lock(mutex);
        history.push_back(enabled);
        return true;
    }
    std::vector<bool> toggles() {
        std::lock_guard<std::mutex> lock(mutex);
        return history;
    }
    bool is_recording_enabled() const override { return recording; }
    std::atomic<bool> recording{false};
    std::atomic<int> toggle_delay_ms{0};
    std::mutex mutex;
    std::vector<bool> history;
};

class RunnerCommandsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        camera_ = std::make_shared<RecordingCamera>();
        ui_state_ = std::make_shared<UIState>();
        auto keypoint_model = std::make_shared<NoopKeypointModel>();
        auto blob_model = std::make_shared<NoopRobotBlobModel>();
        auto loop = std::make_shared<ControlLoop>(
            std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
            std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(), nullptr,
            nullptr, nullptr);
        RunnerRemote remote;
        commands_ = remote.commands;
        runner_ = std::make_unique<Runner>(
            RunnerConfiguration{}, camera_, std::make_shared<HealthLogger>(HealthConfiguration{}),
            std::make_shared<NoopMaskModel>(), blob_model, std::make_shared<NoopFieldFilter>(),
            keypoint_model, std::make_shared<KeypointHeightGate>(KeypointHeightGateConfiguration{}),
            std::make_shared<StaticDetectionGate>(StaticDetectionGateConfiguration{}),
            std::make_shared<ParallelModelBatch>(keypoint_model, blob_model),
            std::make_shared<SteppedControlLoop>(loop, 0.0), std::make_shared<NoopPublisher>(),
            [this](SystemAction action) { system_action_ = action; },
            [this](const std::string &name) { profile_ = name; }, ui_state_, nullptr, nullptr,
            std::move(remote));
    }

    SystemStatus status_after(remote::RemoteCommand command) {
        commands_->post(std::move(command));
        EXPECT_TRUE(runner_->tick());
        SystemStatus status;
        ui_state_->get_system_status(status);
        return status;
    }

    /** Recording toggles run on a worker; tick until the status shows `enabled` or 2 s pass. */
    bool svo_recording_reaches(bool enabled) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            EXPECT_TRUE(runner_->tick());
            SystemStatus status;
            ui_state_->get_system_status(status);
            if (status.svo_recording_enabled == enabled) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    std::shared_ptr<RecordingCamera> camera_;
    std::shared_ptr<UIState> ui_state_;
    std::shared_ptr<remote::CommandQueue> commands_;
    std::unique_ptr<Runner> runner_;
    std::optional<SystemAction> system_action_;
    std::optional<std::string> profile_;
};

TEST_F(RunnerCommandsTest, SetOpponentCount) {
    EXPECT_EQ(status_after(remote::SetOpponentCountCommand{.count = 3}).selected_opponent_count, 3);
    // Out of range is refused, the same as the old atomic path.
    EXPECT_EQ(status_after(remote::SetOpponentCountCommand{.count = 7}).selected_opponent_count, 3);
}

TEST_F(RunnerCommandsTest, SetAutonomyIsAnExplicitSet) {
    EXPECT_FALSE(status_after(remote::SetAutonomyCommand{.enabled = false}).autonomy_enabled);
    // Resending the same value after a reconnect must not flip it back.
    EXPECT_FALSE(status_after(remote::SetAutonomyCommand{.enabled = false}).autonomy_enabled);
    EXPECT_TRUE(status_after(remote::SetAutonomyCommand{.enabled = true}).autonomy_enabled);
}

TEST_F(RunnerCommandsTest, SetRecording) {
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(svo_recording_reaches(true));
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(svo_recording_reaches(true));
    commands_->post(remote::SetRecordingCommand{.enabled = false});
    EXPECT_TRUE(svo_recording_reaches(false));
}

// Stopping a camera's encoder can take seconds on a stalled pipeline. The tick must not wait.
TEST_F(RunnerCommandsTest, SlowRecordingToggleDoesNotBlockTheTick) {
    camera_->toggle_delay_ms = 500;
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    const auto start = std::chrono::steady_clock::now();
    EXPECT_TRUE(runner_->tick());
    const double tick_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    EXPECT_LT(tick_ms, 250.0);
    EXPECT_TRUE(svo_recording_reaches(true));
}

// An off then on from the UI is how a file gets cut; both must apply, in order.
TEST_F(RunnerCommandsTest, OffThenOnBothApply) {
    camera_->recording = true;
    camera_->toggle_delay_ms = 20;
    commands_->post(remote::SetRecordingCommand{.enabled = false});
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(runner_->tick());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (camera_->toggles().size() < 2 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(camera_->toggles(), (std::vector<bool>{false, true}));
}

TEST_F(RunnerCommandsTest, SelectProfilePersistsAndSetsTheNotice) {
    status_after(remote::SelectProfileCommand{.name = "mrs_buff_mk3"});
    EXPECT_EQ(profile_, "mrs_buff_mk3");
    EXPECT_EQ(ui_state_->get_profile_notice(), "Selected mrs_buff_mk3. Reboot to apply.");
}

TEST_F(RunnerCommandsTest, SystemActionStopsTheLoop) {
    camera_->recording = true;
    commands_->post(remote::SystemActionCommand{.action = SystemAction::POWEROFF_HOST});
    EXPECT_FALSE(runner_->tick());
    EXPECT_EQ(system_action_, SystemAction::POWEROFF_HOST);
    // Recordings close before the host goes down.
    EXPECT_FALSE(camera_->recording);
}

TEST_F(RunnerCommandsTest, WifiAccessWithoutHostServicesIsHarmless) {
    status_after(remote::SetWifiAccessCommand{.enabled = true});
    EXPECT_TRUE(runner_->tick());
}

}  // namespace
}  // namespace auto_battlebot
