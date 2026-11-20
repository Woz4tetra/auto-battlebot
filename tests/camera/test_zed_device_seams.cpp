#ifdef BUILD_WITH_ZED
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rgbd_camera/zed_rgbd_camera.hpp"
#include "rgbd_camera/zed_svo_playback_camera.hpp"

namespace auto_battlebot {
namespace {

struct DeviceLog {
    std::mutex mutex;
    std::vector<std::string> events;
    std::atomic<int> grabs{0};
    std::atomic<int> opens{0};
    std::atomic<int> closes{0};

    void record(const std::string &event) {
        std::lock_guard<std::mutex> lock(mutex);
        events.push_back(event);
    }
    std::vector<std::string> snapshot() {
        std::lock_guard<std::mutex> lock(mutex);
        return events;
    }
};

struct FakeBehavior {
    ZedDevice::OpenResult open_result = ZedDevice::OpenResult::Opened;
    ZedDevice::GrabStatus grab_status = ZedDevice::GrabStatus::Ok;
    bool pending_open = false;
    bool await_returns = true;
    bool block_in_grab = false;
    int position = 0;
};

class FakeZedDevice : public ZedDevice {
   public:
    FakeZedDevice(DeviceLog *log, FakeBehavior behavior) : log_(log), behavior_(behavior) {}

    OpenResult open(const std::atomic<bool> &) override {
        ++log_->opens;
        return behavior_.open_result;
    }
    bool await_or_leak_open(const char *, const char *) override {
        log_->record("await");
        return behavior_.await_returns;
    }
    bool has_pending_open() const override { return behavior_.pending_open; }
    void close() override {
        ++log_->closes;
        log_->record("close");
    }
    bool enable_tracking() override { return true; }
    std::optional<CameraInfo> read_camera_info() const override { return CameraInfo{}; }
    GrabStatus grab() override {
        ++log_->grabs;
        if (behavior_.block_in_grab) {
            static std::mutex *never_released = new std::mutex();
            static std::condition_variable *cv = new std::condition_variable();
            std::unique_lock<std::mutex> lock(*never_released);
            cv->wait(lock, [] { return false; });
        }
        return behavior_.grab_status;
    }
    bool retrieve(CameraData &) override { return true; }
    int svo_position() override { return behavior_.position; }

   private:
    DeviceLog *log_;
    FakeBehavior behavior_;
};

ZedSvoPlaybackCameraConfiguration playback_config() {
    ZedSvoPlaybackCameraConfiguration config;
    config.svo_file_path = "unused.svo2";
    config.rebase_stamps = false;
    return config;
}

TEST(ZedSvoPlaybackFakeDeviceTest, AGrabErrorThatIsNotEndOfFileEndsReplayWithoutRetrying) {
    DeviceLog log;
    FakeBehavior behavior;
    behavior.grab_status = ZedDevice::GrabStatus::Fatal;
    ZedSvoPlaybackCamera camera(playback_config(), std::make_unique<FakeZedDevice>(&log, behavior));
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    EXPECT_FALSE(camera.get(data));
    EXPECT_FALSE(camera.get(data));
    EXPECT_FALSE(camera.get(data));
    EXPECT_EQ(log.grabs.load(), 1);
}

TEST(ZedSvoPlaybackFakeDeviceTest, ATransientGrabErrorIsRetried) {
    DeviceLog log;
    FakeBehavior behavior;
    behavior.grab_status = ZedDevice::GrabStatus::TransientError;
    ZedSvoPlaybackCamera camera(playback_config(), std::make_unique<FakeZedDevice>(&log, behavior));
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    EXPECT_FALSE(camera.get(data));
    EXPECT_FALSE(camera.get(data));
    EXPECT_EQ(log.grabs.load(), 2);
}

TEST(ZedSvoPlaybackFakeDeviceTest, FrameIdentityIsThePositionAfterTheGrabMinusOne) {
    DeviceLog log;
    FakeBehavior behavior;
    behavior.position = 5;
    ZedSvoPlaybackCamera camera(playback_config(), std::make_unique<FakeZedDevice>(&log, behavior));
    ASSERT_TRUE(camera.initialize());

    CameraData data;
    ASSERT_TRUE(camera.get(data));
    EXPECT_EQ(data.frame_identity.video_frame_index, 4);
}

TEST(ZedSvoPlaybackFakeDeviceTest, ALeakedOpenFailsInitializeAndClosesNothing) {
    DeviceLog log;
    FakeBehavior behavior;
    behavior.open_result = ZedDevice::OpenResult::Leaked;
    {
        ZedSvoPlaybackCamera camera(playback_config(),
                                    std::make_unique<FakeZedDevice>(&log, behavior));
        EXPECT_FALSE(camera.initialize());
        EXPECT_TRUE(camera.should_close());
        CameraData data;
        EXPECT_FALSE(camera.get(data));
    }
    EXPECT_EQ(log.closes.load(), 0);
    EXPECT_EQ(log.grabs.load(), 0);
}

TEST(ZedSvoPlaybackFakeDeviceTest, CloseWaitsForAPendingOpenAndSkipsCloseWhenItLeaks) {
    {
        DeviceLog log;
        FakeBehavior behavior;
        behavior.pending_open = true;
        behavior.await_returns = true;
        {
            ZedSvoPlaybackCamera camera(playback_config(),
                                        std::make_unique<FakeZedDevice>(&log, behavior));
            ASSERT_TRUE(camera.initialize());
        }
        const std::vector<std::string> expected = {"await", "close"};
        EXPECT_EQ(log.snapshot(), expected);
    }
    {
        DeviceLog log;
        FakeBehavior behavior;
        behavior.pending_open = true;
        behavior.await_returns = false;
        {
            ZedSvoPlaybackCamera camera(playback_config(),
                                        std::make_unique<FakeZedDevice>(&log, behavior));
            ASSERT_TRUE(camera.initialize());
        }
        const std::vector<std::string> expected = {"await"};
        EXPECT_EQ(log.snapshot(), expected);
    }
}

TEST(ZedDeviceOpenTimeoutTest, AWedgedOpenIsLeakedAndDestroyingTheDeviceDoesNotBlock) {
    std::promise<void> release_open;
    std::shared_future<void> gate = release_open.get_future().share();
    std::promise<ZedDevice::OpenResult> result;
    std::future<ZedDevice::OpenResult> result_future = result.get_future();
    std::promise<void> destroyed;
    std::future<void> destroyed_future = destroyed.get_future();

    std::thread lifecycle([&] {
        {
            ZedDevice::Options options;
            options.open_hard_timeout = std::chrono::milliseconds(100);
            options.open_call = [gate] {
                gate.wait();
                return sl::ERROR_CODE::SUCCESS;
            };
            ZedDevice device(std::move(options));
            std::atomic<bool> cancel{true};
            result.set_value(device.open(cancel));
            EXPECT_FALSE(device.has_pending_open());
        }
        destroyed.set_value();
    });

    const bool destroyed_in_time =
        destroyed_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    release_open.set_value();
    lifecycle.join();
    ASSERT_TRUE(destroyed_in_time);
    EXPECT_EQ(result_future.get(), ZedDevice::OpenResult::Leaked);
}

ZedRgbdCameraConfiguration stuck_grab_config() {
    ZedRgbdCameraConfiguration config;
    config.svo_recording = false;
    config.capture_join_timeout_s = 0.2;
    return config;
}

TEST(ZedRgbdFakeDeviceTest, DestructorLeavesTheHandleOpenWhenTheCaptureThreadIsStuckInGrab) {
    auto log = std::make_shared<DeviceLog>();
    FakeBehavior behavior;
    behavior.block_in_grab = true;
    {
        ZedRgbdCamera camera(stuck_grab_config(),
                             std::make_unique<FakeZedDevice>(log.get(), behavior));
        ASSERT_TRUE(camera.initialize());
        while (log->grabs.load() == 0) std::this_thread::yield();
    }
    EXPECT_EQ(log->closes.load(), 0);
    new std::shared_ptr<DeviceLog>(log);
}

TEST(ZedRgbdFakeDeviceTest, InitializeFailsWithoutReopeningWhenTheOldCaptureThreadIsStuck) {
    auto log = std::make_shared<DeviceLog>();
    FakeBehavior behavior;
    behavior.block_in_grab = true;
    {
        ZedRgbdCamera camera(stuck_grab_config(),
                             std::make_unique<FakeZedDevice>(log.get(), behavior));
        ASSERT_TRUE(camera.initialize());
        while (log->grabs.load() == 0) std::this_thread::yield();

        EXPECT_FALSE(camera.initialize());
        EXPECT_TRUE(camera.should_close());
        EXPECT_EQ(log->opens.load(), 1);
    }
    EXPECT_EQ(log->closes.load(), 0);
    new std::shared_ptr<DeviceLog>(log);
}

}  // namespace
}  // namespace auto_battlebot
#endif
