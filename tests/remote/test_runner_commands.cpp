#include <gtest/gtest.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "control_loop/stepped_control_loop.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "field_filter/noop_field_filter.hpp"
#include "keypoint_model/noop_keypoint_model.hpp"
#include "mask_model/noop_mask_model.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
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

// [control_loop] robot_cache_max_age_s default.
constexpr double kCacheMaxAgeS = 0.25;

// Toggled on the runner's recording worker and read by the tick, hence the atomics.
class RecordingCamera : public NoopRgbdCamera {
   public:
    bool set_recording_enabled(bool enabled) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(toggle_delay_ms.load()));
        if (opening.load()) overlapped_open = true;
        if (on_toggle) on_toggle(enabled);
        recording = enabled;
        // Like VideoEncoder: a restart clears a stopped encoder.
        if (enabled) needs_restart = false;
        std::lock_guard<std::mutex> lock(mutex);
        history.push_back(enabled);
        return true;
    }
    std::vector<bool> toggles() {
        std::lock_guard<std::mutex> lock(mutex);
        return history;
    }
    bool is_recording_enabled() const override { return recording; }
    bool recording_encoder_stalled() const override { return needs_restart; }
    std::atomic<bool> needs_restart{false};
    std::atomic<bool> overlapped_open{false};
    std::function<void(bool)> on_toggle;

    // With lost_ set, get() fails and a re-open blocks like a ZED that is not there, until
    // cancel_initialize() or 10 s.
    bool get(CameraData &data) override { return !lost_.load() && NoopRgbdCamera::get(data); }
    bool initialize() override {
        if (!lost_.load()) return true;
        if (open_fails_.load()) return false;
        cancel_.store(false);
        opening.store(true);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!cancel_.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        opening.store(false);
        return false;
    }
    void cancel_initialize() override { cancel_.store(true); }
    std::atomic<bool> lost_{false};
    std::atomic<bool> open_fails_{false};
    std::atomic<bool> opening{false};
    std::atomic<bool> cancel_{false};

    std::atomic<bool> recording{false};
    std::atomic<int> toggle_delay_ms{0};
    std::mutex mutex;
    std::vector<bool> history;
};

// Records what the runner queues, and echoes it back as the read-back the way the capture thread
// does a frame later. DEFAULT (nullopt) restores `opened`.
class SettingsCamera : public NoopRgbdCamera {
   public:
    bool supports_camera_settings() const override { return true; }
    void request_camera_settings(std::optional<CameraSettings> settings) override {
        requests.push_back(settings);
        current = settings.value_or(opened);
    }
    std::optional<CameraSettingsState> camera_settings() const override {
        return CameraSettingsState{current, {}};
    }
    CameraSettings opened;
    CameraSettings current;
    std::vector<std::optional<CameraSettings>> requests;
};

class RunnerCommandsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        camera_ = std::make_shared<RecordingCamera>();
        auto keypoint_model = std::make_shared<NoopKeypointModel>();
        auto blob_model = std::make_shared<NoopRobotBlobModel>();
        auto loop = std::make_shared<ControlLoop>(
            std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
            std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(), nullptr,
            nullptr, kCacheMaxAgeS);
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
            [this](const std::string &name) { profile_ = name; }, nullptr, nullptr,
            std::move(remote));
    }

    remote::SystemStatusMessage status_after(remote::RemoteCommand command) {
        commands_->post(std::move(command));
        EXPECT_TRUE(runner_->tick());
        return runner_->system_status();
    }

    /** Recording toggles run on a worker; tick until the status shows `enabled` or 2 s pass. */
    bool camera_recording_reaches(bool enabled) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            EXPECT_TRUE(runner_->tick());
            if (runner_->system_status().camera_recording == enabled) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    std::shared_ptr<RecordingCamera> camera_;
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

TEST_F(RunnerCommandsTest, SetBehaviorMode) {
    EXPECT_EQ(runner_->system_status().behavior_mode, BehaviorMode::ATTACK);
    EXPECT_EQ(
        status_after(remote::SetBehaviorModeCommand{.mode = BehaviorMode::RUN_AWAY}).behavior_mode,
        BehaviorMode::RUN_AWAY);
    EXPECT_EQ(
        status_after(remote::SetBehaviorModeCommand{.mode = BehaviorMode::ATTACK}).behavior_mode,
        BehaviorMode::ATTACK);
}

TEST_F(RunnerCommandsTest, SetAutoTracking) {
    EXPECT_TRUE(status_after(remote::SetAutoTrackingCommand{.enabled = true}).auto_tracking);
    EXPECT_FALSE(status_after(remote::SetAutoTrackingCommand{.enabled = false}).auto_tracking);
    EXPECT_TRUE(status_after(remote::SetAutoTrackingCommand{.enabled = true}).auto_tracking);
}

TEST_F(RunnerCommandsTest, ManualTargetNeverChangesAutoTracking) {
    // Only the switch does. A tap while auto tracking is on is refused, not a takeover.
    remote::SetManualTargetCommand command;
    command.field = remote::FieldPoint{.x = 0.5, .y = -0.25};
    EXPECT_TRUE(status_after(command).auto_tracking);
    EXPECT_FALSE(status_after(remote::SetAutoTrackingCommand{.enabled = false}).auto_tracking);
    EXPECT_FALSE(status_after(command).auto_tracking);
}

TEST_F(RunnerCommandsTest, SetRecording) {
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(camera_recording_reaches(true));
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(camera_recording_reaches(true));
    commands_->post(remote::SetRecordingCommand{.enabled = false});
    EXPECT_TRUE(camera_recording_reaches(false));
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
    EXPECT_TRUE(camera_recording_reaches(true));
}

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

TEST_F(RunnerCommandsTest, StoppedEncoderRestartsTheRecording) {
    camera_->recording = true;
    camera_->needs_restart = true;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (camera_->toggles().size() < 2 && std::chrono::steady_clock::now() < deadline) {
        EXPECT_TRUE(runner_->tick());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(camera_->toggles(), (std::vector<bool>{false, true}));
    EXPECT_FALSE(camera_->needs_restart);
}

TEST_F(RunnerCommandsTest, SelectProfilePersists) {
    status_after(remote::SelectProfileCommand{.name = "mrs_buff_mk3"});
    EXPECT_EQ(profile_, "mrs_buff_mk3");
}

TEST_F(RunnerCommandsTest, SystemActionStopsTheLoop) {
    camera_->recording = true;
    commands_->post(remote::SystemActionCommand{.action = SystemAction::POWEROFF_HOST});
    EXPECT_FALSE(runner_->tick());
    EXPECT_EQ(system_action_, SystemAction::POWEROFF_HOST);
    // Recordings close before the host goes down.
    EXPECT_FALSE(camera_->recording);
}

class LostCameraTick {
   public:
    LostCameraTick(Runner &runner, RecordingCamera &camera) {
        camera.lost_ = true;
        thread_ = std::thread([&] { result_ = runner.tick(); });
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!camera.opening.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        EXPECT_TRUE(camera.opening.load()) << "the tick never started re-opening the camera";
        start_ = std::chrono::steady_clock::now();
    }
    /** Milliseconds from the re-open starting to the tick returning. */
    double join() {
        thread_.join();
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_)
            .count();
    }
    bool result() const { return result_; }

   private:
    std::thread thread_;
    std::atomic<bool> result_{true};
    std::chrono::steady_clock::time_point start_;
};

TEST_F(RunnerCommandsTest, QuitCancelsACameraReopen) {
    LostCameraTick tick(*runner_, *camera_);
    runner_->quit_from_signal_handler();
    EXPECT_LT(tick.join(), 500.0);
    EXPECT_FALSE(tick.result());
}

TEST_F(RunnerCommandsTest, RebootCancelsACameraReopenAndStillRuns) {
    camera_->recording = true;
    LostCameraTick tick(*runner_, *camera_);
    commands_->post(remote::SystemActionCommand{.action = SystemAction::REBOOT_HOST});
    EXPECT_LT(tick.join(), 500.0);
    EXPECT_FALSE(tick.result());
    EXPECT_EQ(system_action_, SystemAction::REBOOT_HOST);
    EXPECT_FALSE(camera_->recording);
}

// A camera that cannot be re-opened ends the run. The recordings must be closed by then, since
// the camera's destructor may have to exit the process.
TEST_F(RunnerCommandsTest, CameraFailureExitClosesRecordings) {
    camera_->recording = true;
    camera_->lost_ = true;
    camera_->open_fails_ = true;
    EXPECT_EQ(runner_->run(), 0);
    EXPECT_FALSE(camera_->recording);
}

TEST_F(RunnerCommandsTest, WifiAccessWithoutHostServicesIsHarmless) {
    status_after(remote::SetWifiAccessCommand{.enabled = true});
    EXPECT_TRUE(runner_->tick());
}

TEST_F(RunnerCommandsTest, CameraWithoutLiveControlsRefusesSettings) {
    commands_->post(remote::SetCameraSettingsCommand{});
    EXPECT_TRUE(runner_->tick());
    EXPECT_FALSE(runner_->camera_settings_status().supported);
    EXPECT_EQ(runner_->camera_settings_status().profiles, std::vector<std::string>{"DEFAULT"});
}

class CameraSettingsCommandsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() / "auto_battlebot_runner_camera_test";
        std::filesystem::remove_all(dir_);
        camera_ = std::make_shared<SettingsCamera>();
        camera_->opened.exposure_us = 8000;
        camera_->current = camera_->opened;
        auto keypoint_model = std::make_shared<NoopKeypointModel>();
        auto blob_model = std::make_shared<NoopRobotBlobModel>();
        auto loop = std::make_shared<ControlLoop>(
            std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
            std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(), nullptr,
            nullptr, kCacheMaxAgeS);
        RunnerRemote remote;
        commands_ = remote.commands;
        store_ = std::make_shared<CameraProfileStore>(dir_ / "camera_profiles.toml");
        store_->load();
        remote.camera_profiles = store_;
        runner_ = std::make_unique<Runner>(
            RunnerConfiguration{}, camera_, std::make_shared<HealthLogger>(HealthConfiguration{}),
            std::make_shared<NoopMaskModel>(), blob_model, std::make_shared<NoopFieldFilter>(),
            keypoint_model, std::make_shared<KeypointHeightGate>(KeypointHeightGateConfiguration{}),
            std::make_shared<StaticDetectionGate>(StaticDetectionGateConfiguration{}),
            std::make_shared<ParallelModelBatch>(keypoint_model, blob_model),
            std::make_shared<SteppedControlLoop>(loop, 0.0), std::make_shared<NoopPublisher>(),
            [](SystemAction) {}, [](const std::string &) {}, nullptr, nullptr, std::move(remote));
    }
    void TearDown() override { std::filesystem::remove_all(dir_); }

    remote::CameraSettingsMessage after(remote::RemoteCommand command) {
        commands_->post(std::move(command));
        EXPECT_TRUE(runner_->tick());
        return runner_->camera_settings_status();
    }

    static CameraSettings manual(int exposure_us) {
        CameraSettings s;
        s.auto_exposure = false;
        s.exposure_us = exposure_us;
        return s;
    }

    std::filesystem::path dir_;
    std::shared_ptr<SettingsCamera> camera_;
    std::shared_ptr<CameraProfileStore> store_;
    std::shared_ptr<remote::CommandQueue> commands_;
    std::unique_ptr<Runner> runner_;
};

TEST_F(CameraSettingsCommandsTest, StartsOnDefaultWithoutTouchingTheCamera) {
    store_->save("cage", manual(3000));
    const auto status = runner_->camera_settings_status();
    EXPECT_TRUE(status.supported);
    EXPECT_EQ(status.active_profile, "DEFAULT");
    EXPECT_FALSE(status.modified);
    EXPECT_EQ(status.profiles.front(), "DEFAULT");
    EXPECT_TRUE(camera_->requests.empty());
}

TEST_F(CameraSettingsCommandsTest, LiveEditMarksModifiedAndSaveStoresIt) {
    auto status = after(remote::SetCameraSettingsCommand{.settings = manual(2500)});
    EXPECT_TRUE(status.modified);
    EXPECT_EQ(status.active_profile, "DEFAULT");
    EXPECT_EQ(status.settings.exposure_us, 2500);

    status =
        after(remote::CameraProfileCommand{.action = CameraProfileAction::SAVE, .name = "cage"});
    EXPECT_FALSE(status.modified);
    EXPECT_EQ(status.active_profile, "cage");
    EXPECT_EQ(status.profiles, (std::vector<std::string>{"DEFAULT", "cage"}));
    EXPECT_EQ(store_->settings("cage")->exposure_us, 2500);
}

TEST_F(CameraSettingsCommandsTest, ApplyDefaultRestoresWhatTheCameraOpenedWith) {
    store_->save("cage", manual(3000));
    auto status =
        after(remote::CameraProfileCommand{.action = CameraProfileAction::APPLY, .name = "cage"});
    EXPECT_EQ(status.settings.exposure_us, 3000);
    EXPECT_EQ(status.active_profile, "cage");

    status = after(
        remote::CameraProfileCommand{.action = CameraProfileAction::APPLY, .name = "DEFAULT"});
    EXPECT_EQ(status.active_profile, "DEFAULT");
    ASSERT_FALSE(camera_->requests.empty());
    EXPECT_FALSE(camera_->requests.back().has_value());
    EXPECT_EQ(status.settings, camera_->opened);
}

TEST_F(CameraSettingsCommandsTest, RefusalsLeaveTheActiveProfile) {
    auto status = after(
        remote::CameraProfileCommand{.action = CameraProfileAction::APPLY, .name = "missing"});
    EXPECT_EQ(status.active_profile, "DEFAULT");
    status =
        after(remote::CameraProfileCommand{.action = CameraProfileAction::SAVE, .name = "DEFAULT"});
    EXPECT_EQ(status.profiles, std::vector<std::string>{"DEFAULT"});
    EXPECT_TRUE(camera_->requests.empty());
}

TEST_F(CameraSettingsCommandsTest, DeletingTheActiveProfileFallsBackToDefaultLabel) {
    store_->save("cage", manual(3000));
    after(remote::CameraProfileCommand{.action = CameraProfileAction::APPLY, .name = "cage"});
    const auto status =
        after(remote::CameraProfileCommand{.action = CameraProfileAction::DELETE, .name = "cage"});
    EXPECT_EQ(status.active_profile, "DEFAULT");
    // The camera still runs cage's values, which differ from DEFAULT's.
    EXPECT_TRUE(status.modified);
    EXPECT_EQ(status.profiles, std::vector<std::string>{"DEFAULT"});
}

class FlushRecordingPublisher : public NoopPublisher {
   public:
    void flush() override {
        if (on_flush) on_flush();
    }
    std::function<void()> on_flush;
};

class StalledControlLoop : public SteppedControlLoop {
   public:
    using SteppedControlLoop::SteppedControlLoop;
    bool is_healthy() const override { return false; }
};

class RunnerRecordingOrderTest : public ::testing::Test {
   protected:
    void SetUp() override {
        camera_ = std::make_shared<RecordingCamera>();
        publisher_ = std::make_shared<FlushRecordingPublisher>();
        recorder_ = std::make_shared<McapRecorder>("runner_order_test");
        paths_.push_back(recorder_->file_path());
        camera_->on_toggle = [this](bool enabled) {
            note(std::string(enabled ? "svo_on" : "svo_off") +
                 (recorder_->is_enabled() ? "_mcap_on" : "_mcap_off"));
        };
        publisher_->on_flush = [this] {
            note(std::string("flush") + (recorder_->is_enabled() ? "_mcap_on" : "_mcap_off"));
        };
        auto keypoint_model = std::make_shared<NoopKeypointModel>();
        auto blob_model = std::make_shared<NoopRobotBlobModel>();
        auto loop = std::make_shared<ControlLoop>(
            std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
            std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(), nullptr,
            nullptr, kCacheMaxAgeS);
        RunnerRemote remote;
        commands_ = remote.commands;
        runner_ = std::make_unique<Runner>(
            RunnerConfiguration{}, camera_, std::make_shared<HealthLogger>(HealthConfiguration{}),
            std::make_shared<NoopMaskModel>(), blob_model, std::make_shared<NoopFieldFilter>(),
            keypoint_model, std::make_shared<KeypointHeightGate>(KeypointHeightGateConfiguration{}),
            std::make_shared<StaticDetectionGate>(StaticDetectionGateConfiguration{}),
            std::make_shared<ParallelModelBatch>(keypoint_model, blob_model),
            std::make_shared<SteppedControlLoop>(loop, 0.0), publisher_, [](SystemAction) {},
            nullptr, recorder_, nullptr, std::move(remote));
    }

    void TearDown() override {
        recorder_->close();
        std::error_code ec;
        for (const auto &path : paths_) std::filesystem::remove(path, ec);
    }

    void note(const std::string &event) {
        std::lock_guard<std::mutex> lock(events_mutex_);
        events_.push_back(event);
    }

    std::vector<std::string> events() {
        std::lock_guard<std::mutex> lock(events_mutex_);
        return events_;
    }

    bool tick_until_events(size_t count) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < deadline) {
            EXPECT_TRUE(runner_->tick());
            if (events().size() >= count) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    std::shared_ptr<RecordingCamera> camera_;
    std::shared_ptr<FlushRecordingPublisher> publisher_;
    std::shared_ptr<McapRecorder> recorder_;
    std::shared_ptr<remote::CommandQueue> commands_;
    std::unique_ptr<Runner> runner_;
    std::vector<std::filesystem::path> paths_;
    std::mutex events_mutex_;
    std::vector<std::string> events_;
};

TEST_F(RunnerRecordingOrderTest, RecordingOnOpensMcapBeforeTheEncoderStarts) {
    recorder_->set_enabled(false);
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    ASSERT_TRUE(tick_until_events(1));
    EXPECT_EQ(events(), (std::vector<std::string>{"svo_on_mcap_on"}));
    paths_.push_back(recorder_->file_path());
}

TEST_F(RunnerRecordingOrderTest, RecordingOffStopsTheEncoderThenDrainsThePublisherThenClosesMcap) {
    commands_->post(remote::SetRecordingCommand{.enabled = false});
    ASSERT_TRUE(tick_until_events(2));
    EXPECT_EQ(events(), (std::vector<std::string>{"svo_off_mcap_on", "flush_mcap_on"}));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (recorder_->is_enabled() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_FALSE(recorder_->is_enabled());
}

TEST_F(RunnerRecordingOrderTest, SystemActionFlushesThePublisherBeforeClosingMcap) {
    commands_->post(remote::SystemActionCommand{.action = SystemAction::REBOOT_HOST});
    EXPECT_FALSE(runner_->tick());
    const auto recorded = events();
    ASSERT_FALSE(recorded.empty());
    EXPECT_EQ(recorded.front(), "svo_off_mcap_on");
    EXPECT_NE(std::find(recorded.begin(), recorded.end(), "flush_mcap_on"), recorded.end());
    EXPECT_FALSE(recorder_->is_enabled());
}

TEST_F(RunnerRecordingOrderTest, ShutdownWaitsForAToggleAlreadyPosted) {
    camera_->toggle_delay_ms = 200;
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(runner_->tick());
    commands_->post(remote::SystemActionCommand{.action = SystemAction::POWEROFF_HOST});
    EXPECT_FALSE(runner_->tick());
    EXPECT_EQ(camera_->toggles(), (std::vector<bool>{true, false}));
    EXPECT_FALSE(camera_->recording);
}

TEST_F(RunnerRecordingOrderTest, RecordingToggleWaitsOutACameraReopen) {
    camera_->toggle_delay_ms = 400;
    commands_->post(remote::SetRecordingCommand{.enabled = true});
    EXPECT_TRUE(runner_->tick());
    camera_->lost_ = true;
    std::atomic<bool> result{true};
    std::thread tick([&] { result = runner_->tick(); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (camera_->toggles().empty() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    runner_->quit_from_signal_handler();
    tick.join();
    EXPECT_FALSE(camera_->overlapped_open.load());
}

TEST_F(RunnerCommandsTest, StalledControlLoopCutsAutonomyAndLogsOnce) {
    if (!DiagnosticsLogger::is_initialized()) DiagnosticsLogger::initialize({});
    auto keypoint_model = std::make_shared<NoopKeypointModel>();
    auto blob_model = std::make_shared<NoopRobotBlobModel>();
    auto loop = std::make_shared<ControlLoop>(
        std::make_shared<NoopRobotFilter>(), std::make_shared<NoopTarget>(),
        std::make_shared<NoopNavigation>(), std::make_shared<NoopTransmitter>(), nullptr, nullptr,
        kCacheMaxAgeS);
    Runner stalled(
        RunnerConfiguration{}, camera_, std::make_shared<HealthLogger>(HealthConfiguration{}),
        std::make_shared<NoopMaskModel>(), blob_model, std::make_shared<NoopFieldFilter>(),
        keypoint_model, std::make_shared<KeypointHeightGate>(KeypointHeightGateConfiguration{}),
        std::make_shared<StaticDetectionGate>(StaticDetectionGateConfiguration{}),
        std::make_shared<ParallelModelBatch>(keypoint_model, blob_model),
        std::make_shared<StalledControlLoop>(loop, 0.0), std::make_shared<NoopPublisher>(),
        [](SystemAction) {}, nullptr, nullptr, nullptr, RunnerRemote{});

    std::atomic<int> errors{0};
    const auto previous = spdlog::default_logger();
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "stalled", std::make_shared<spdlog::sinks::callback_sink_mt>(
                       [&errors](const spdlog::details::log_msg &msg) {
                           if (msg.level == spdlog::level::err) ++errors;
                       })));
    std::thread runner_thread([&] { stalled.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    stalled.quit_from_signal_handler();
    runner_thread.join();
    spdlog::set_default_logger(previous);

    EXPECT_FALSE(stalled.system_status().autonomy_enabled);
    EXPECT_EQ(errors.load(), 1);
}

}  // namespace
}  // namespace auto_battlebot
