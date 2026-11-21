#ifdef BUILD_WITH_ZED
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <mcap/reader.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "../scratch_recordings.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "rgbd_camera/h264_backend.hpp"
#include "rgbd_camera/zed_one_rgb_camera.hpp"

namespace auto_battlebot {
namespace {

constexpr int kWidth = 8;
constexpr int kHeight = 4;

struct OneState {
    std::mutex mutex;
    std::vector<std::pair<sl::VIDEO_SETTINGS, int>> writes;
    std::map<int, int> effective;
    std::map<int, int> last_written;
    bool unknown_exposure_and_gains = false;
    std::atomic<int> grabs{0};
    std::atomic<int> closes{0};
    std::atomic<bool> block_grab{false};
    std::atomic<bool> open_entered{false};
    std::atomic<bool> open_released{true};
    std::atomic<bool> open_returned{false};
    std::atomic<bool> closed_before_open_returned{false};

    std::vector<int> writes_to(sl::VIDEO_SETTINGS setting) {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<int> values;
        for (const auto &[s, v] : writes) {
            if (s == setting) values.push_back(v);
        }
        return values;
    }
    int effective_value(sl::VIDEO_SETTINGS setting) {
        std::lock_guard<std::mutex> lock(mutex);
        return effective[static_cast<int>(setting)];
    }
    void forget_effective(sl::VIDEO_SETTINGS setting, int value) {
        std::lock_guard<std::mutex> lock(mutex);
        effective[static_cast<int>(setting)] = value;
    }
};

class FakeZedOneDevice : public ZedOneDevice {
   public:
    explicit FakeZedOneDevice(std::shared_ptr<OneState> state) : state_(std::move(state)) {}

    sl::ERROR_CODE open(const sl::InitParametersOne &) override {
        state_->open_entered = true;
        while (!state_->open_released) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        state_->open_returned = true;
        return sl::ERROR_CODE::SUCCESS;
    }
    void close() override {
        if (!state_->open_returned) state_->closed_before_open_returned = true;
        ++state_->closes;
    }
    sl::ERROR_CODE grab() override {
        if (state_->block_grab) {
            std::mutex *never = new std::mutex();
            std::condition_variable *cv = new std::condition_variable();
            std::unique_lock<std::mutex> lock(*never);
            cv->wait(lock, [] { return false; });
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        ++state_->grabs;
        return sl::ERROR_CODE::SUCCESS;
    }
    uint64_t image_timestamp_ns() override {
        return 1'000'000'000ULL * static_cast<uint64_t>(state_->grabs.load());
    }
    std::optional<cv::Mat> retrieve_bgra() override {
        buffer_.create(kHeight, kWidth, CV_8UC4);
        buffer_.setTo(cv::Scalar(state_->grabs.load() % 256, 20, 30, 255));
        return buffer_;
    }
    sl::CameraOneInformation information() override {
        sl::CameraOneInformation info;
        info.serial_number = 1234;
        info.camera_configuration.resolution.width = kWidth;
        info.camera_configuration.resolution.height = kHeight;
        info.camera_configuration.fps = 60;
        info.camera_configuration.calibration_parameters.fx = 700.0f;
        info.camera_configuration.calibration_parameters.fy = 710.0f;
        info.camera_configuration.calibration_parameters.cx = 4.0f;
        info.camera_configuration.calibration_parameters.cy = 2.0f;
        info.camera_configuration.calibration_parameters.disto[0] = 0.25;
        return info;
    }
    bool is_setting_supported(sl::VIDEO_SETTINGS) override { return true; }
    sl::ERROR_CODE get_setting(sl::VIDEO_SETTINGS setting, int &value) override {
        const bool gain_or_exposure = setting == sl::VIDEO_SETTINGS::EXPOSURE_TIME ||
                                      setting == sl::VIDEO_SETTINGS::ANALOG_GAIN ||
                                      setting == sl::VIDEO_SETTINGS::DIGITAL_GAIN;
        if (state_->unknown_exposure_and_gains && gain_or_exposure) {
            value = -1;
            return sl::ERROR_CODE::SUCCESS;
        }
        std::lock_guard<std::mutex> lock(state_->mutex);
        value = state_->effective[static_cast<int>(setting)];
        return sl::ERROR_CODE::SUCCESS;
    }
    sl::ERROR_CODE get_range(sl::VIDEO_SETTINGS setting, int &min, int &max) override {
        switch (setting) {
            case sl::VIDEO_SETTINGS::EXPOSURE_TIME:
            case sl::VIDEO_SETTINGS::ANALOG_GAIN:
                min = 28;
                max = 30000;
                break;
            case sl::VIDEO_SETTINGS::DIGITAL_GAIN:
                min = 1;
                max = 16;
                break;
            case sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE:
                min = 2800;
                max = 6500;
                break;
            default:
                min = 0;
                max = 8;
                break;
        }
        return sl::ERROR_CODE::SUCCESS;
    }
    sl::ERROR_CODE set_setting(sl::VIDEO_SETTINGS setting, int value) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->writes.emplace_back(setting, value);
        const int key = static_cast<int>(setting);
        const auto last = state_->last_written.find(key);
        if (last != state_->last_written.end() && last->second == value) {
            return sl::ERROR_CODE::SUCCESS;
        }
        state_->last_written[key] = value;
        state_->effective[key] = value;
        return sl::ERROR_CODE::SUCCESS;
    }

   private:
    std::shared_ptr<OneState> state_;
    cv::Mat buffer_;
};

class OneBytePerFrameBackend : public H264Backend {
   public:
    explicit OneBytePerFrameBackend(VideoEncoder::BackendSink sink) : sink_(std::move(sink)) {}
    bool encode(cv::Mat, int64_t pts, bool) override {
        const std::byte byte{0};
        sink_(&byte, 1, pts, pts == 0);
        return true;
    }
    void finish() override {}
    const std::string &name() const override { return name_; }

   private:
    VideoEncoder::BackendSink sink_;
    std::string name_ = "one_byte";
};

VideoEncoder::BackendFactory one_byte_factory() {
    return [](const VideoEncoderOptions &, VideoEncoder::BackendSink sink) {
        return std::unique_ptr<H264Backend>(new OneBytePerFrameBackend(std::move(sink)));
    };
}

VideoEncoder::BackendFactory refusing_factory() {
    return [](const VideoEncoderOptions &, VideoEncoder::BackendSink) {
        return std::unique_ptr<H264Backend>();
    };
}

template <typename Pred>
bool wait_until(Pred pred, int timeout_ms = 3000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

ZedOneRgbCameraConfiguration make_config(bool recording) {
    ZedOneRgbCameraConfiguration config;
    config.video_recording = recording;
    return config;
}

std::unique_ptr<ZedOneRgbCamera> make_camera(std::shared_ptr<OneState> state,
                                             bool recording = false,
                                             std::shared_ptr<McapRecorder> recorder = nullptr,
                                             VideoEncoder::BackendFactory factory = nullptr) {
    return std::make_unique<ZedOneRgbCamera>(make_config(recording), std::move(recorder),
                                             std::make_unique<FakeZedOneDevice>(std::move(state)),
                                             std::move(factory));
}

CameraSettings manual_settings(int exposure_us) {
    CameraSettings settings;
    settings.auto_exposure = false;
    settings.exposure_us = exposure_us;
    settings.analog_gain_mdb = 100;
    settings.digital_gain = 2;
    return settings;
}

bool applied(const std::shared_ptr<OneState> &state, int exposure_us) {
    return wait_until(
        [&] { return state->effective_value(sl::VIDEO_SETTINGS::EXPOSURE_TIME) == exposure_us; });
}

TEST(ZedOneFakeDeviceTest, SettingsReadBackKeepsTheAppliedValueWhenTheSdkReportsMinusOne) {
    auto state = std::make_shared<OneState>();
    state->unknown_exposure_and_gains = true;
    auto camera = make_camera(state);
    ASSERT_TRUE(camera->initialize());
    camera->request_camera_settings(manual_settings(5000));
    ASSERT_TRUE(applied(state, 5000));
    const int target = state->grabs.load() + 70;
    ASSERT_TRUE(wait_until([&] { return state->grabs.load() >= target; }));
    const auto reported = camera->camera_settings();
    ASSERT_TRUE(reported.has_value());
    EXPECT_EQ(reported->settings.exposure_us, 5000);
    EXPECT_EQ(reported->settings.analog_gain_mdb, 100);
    EXPECT_EQ(reported->settings.digital_gain, 2);
    for (int value : state->writes_to(sl::VIDEO_SETTINGS::EXPOSURE_TIME)) EXPECT_GE(value, 28);
}

TEST(ZedOneFakeDeviceTest, ARepeatedProfileStillLandsAfterTheSdkSkippedAnEqualWrite) {
    auto state = std::make_shared<OneState>();
    auto camera = make_camera(state);
    ASSERT_TRUE(camera->initialize());
    camera->request_camera_settings(manual_settings(5000));
    ASSERT_TRUE(applied(state, 5000));
    state->forget_effective(sl::VIDEO_SETTINGS::EXPOSURE_TIME, 100);
    camera->request_camera_settings(manual_settings(5000));
    EXPECT_TRUE(applied(state, 5000));
}

TEST(ZedOneFakeDeviceTest, EachRealWriteIsPrecededByAnAdjacentValueExceptTheOnOffSwitches) {
    auto state = std::make_shared<OneState>();
    auto camera = make_camera(state);
    ASSERT_TRUE(camera->initialize());
    camera->request_camera_settings(manual_settings(5000));
    ASSERT_TRUE(applied(state, 5000));
    EXPECT_EQ(state->writes_to(sl::VIDEO_SETTINGS::EXPOSURE_TIME), (std::vector<int>{5001, 5000}));
    EXPECT_EQ(state->writes_to(sl::VIDEO_SETTINGS::AEC_AGC), (std::vector<int>{0}));
    camera->request_camera_settings(manual_settings(16666));
    ASSERT_TRUE(applied(state, 16666));
    const auto exposure = state->writes_to(sl::VIDEO_SETTINGS::EXPOSURE_TIME);
    ASSERT_GE(exposure.size(), 4u);
    EXPECT_EQ(exposure[exposure.size() - 2], 16665);
}

TEST(ZedOneFakeDeviceTest, AntiBandingIsWrittenWhenTheSdkHasTheControl) {
    auto state = std::make_shared<OneState>();
    auto camera = make_camera(state);
    ASSERT_TRUE(camera->initialize());
    CameraSettings settings = manual_settings(5000);
    settings.anti_banding = AntiBanding::HZ_50;
    camera->request_camera_settings(settings);
    EXPECT_TRUE(wait_until(
        [&] { return state->effective_value(sl::VIDEO_SETTINGS::AE_ANTIBANDING) == 2; }));
}

TEST(ZedOneFakeDeviceTest, FramesAreBgrCopiesWithZeroDistortionAndMatchingHeaders) {
    auto state = std::make_shared<OneState>();
    auto camera = make_camera(state);
    ASSERT_TRUE(camera->initialize());
    CameraData data;
    ASSERT_TRUE(wait_until([&] { return camera->get(data); }));
    EXPECT_EQ(data.rgb.image.type(), CV_8UC3);
    EXPECT_EQ(data.rgb.image.cols, kWidth);
    EXPECT_EQ(data.rgb.image.at<cv::Vec3b>(1, 1)[1], 20);
    EXPECT_EQ(data.rgb.image.at<cv::Vec3b>(1, 1)[2], 30);
    EXPECT_EQ(data.rgb.image.at<cv::Vec3b>(1, 1)[0], static_cast<int>(data.rgb.header.stamp) % 256);
    EXPECT_EQ(cv::countNonZero(data.camera_info.distortion), 0);
    EXPECT_EQ(data.camera_info.distortion.cols, 5);
    EXPECT_DOUBLE_EQ(data.camera_info.intrinsics.at<double>(0, 0), 700.0);
    EXPECT_EQ(data.camera_info.header.stamp, data.rgb.header.stamp);
    EXPECT_EQ(data.camera_info.header.frame_id, data.rgb.header.frame_id);
}

TEST(ZedOneFakeDeviceTest, EachFrameIsAFreshMatThatLaterCapturesDoNotOverwrite) {
    auto state = std::make_shared<OneState>();
    auto camera = make_camera(state);
    ASSERT_TRUE(camera->initialize());
    CameraData first;
    ASSERT_TRUE(wait_until([&] { return camera->get(first); }));
    const int first_blue = first.rgb.image.at<cv::Vec3b>(0, 0)[0];
    CameraData second;
    ASSERT_TRUE(wait_until(
        [&] { return camera->get(second) && second.rgb.header.stamp > first.rgb.header.stamp; }));
    EXPECT_NE(first.rgb.image.data, second.rgb.image.data);
    EXPECT_EQ(first.rgb.image.at<cv::Vec3b>(0, 0)[0], first_blue);
    EXPECT_NE(second.rgb.image.at<cv::Vec3b>(0, 0)[0], first_blue);
}

TEST(ZedOneFakeDeviceTest, CancellingOpenWaitsForOpenToReturnBeforeClosing) {
    auto state = std::make_shared<OneState>();
    state->open_released = false;
    auto camera = make_camera(state);
    std::atomic<bool> finished{false};
    bool result = true;
    std::thread initializer([&] {
        result = camera->initialize();
        finished = true;
    });
    ASSERT_TRUE(wait_until([&] { return state->open_entered.load(); }));
    camera->cancel_initialize();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_FALSE(finished.load());
    EXPECT_EQ(state->closes.load(), 0);
    state->open_released = true;
    initializer.join();
    EXPECT_FALSE(result);
    EXPECT_EQ(state->closes.load(), 1);
    EXPECT_FALSE(state->closed_before_open_returned.load());
}

TEST(ZedOneFakeDeviceTest, RecordingOnWithoutAnEncoderFailsInitialize) {
    const auto dir = make_scratch_recordings_dir("zed_one_no_encoder");
    auto recorder = std::make_shared<McapRecorder>("zed_one_test", dir);
    auto camera = make_camera(std::make_shared<OneState>(), true, recorder, refusing_factory());
    EXPECT_THROW(camera->initialize(), std::runtime_error);
}

TEST(ZedOneFakeDeviceTest, RecordingOffInitializesWithoutAnEncoder) {
    const auto dir = make_scratch_recordings_dir("zed_one_recording_off");
    auto recorder = std::make_shared<McapRecorder>("zed_one_test", dir);
    auto camera = make_camera(std::make_shared<OneState>(), false, recorder, refusing_factory());
    EXPECT_TRUE(camera->initialize());
}

TEST(ZedOneFakeDeviceTest, VideoMessagesAreLoggedAtStrictlyIncreasingTimes) {
    const auto dir = make_scratch_recordings_dir("zed_one_video_times");
    auto recorder = std::make_shared<McapRecorder>("zed_one_test", dir);
    auto state = std::make_shared<OneState>();
    {
        auto camera = make_camera(state, true, recorder, one_byte_factory());
        ASSERT_TRUE(camera->initialize());
        CameraData data;
        int received = 0;
        ASSERT_TRUE(wait_until([&] {
            if (camera->get(data)) ++received;
            return received >= 20;
        }));
    }
    recorder->close();
    mcap::McapReader reader;
    ASSERT_TRUE(reader.open(recorder->file_path().string()).ok());
    std::vector<uint64_t> log_times;
    for (const auto &view : reader.readMessages()) {
        if (view.channel->topic == "/camera/video") log_times.push_back(view.message.logTime);
    }
    reader.close();
    ASSERT_GE(log_times.size(), 10u);
    for (size_t i = 1; i < log_times.size(); ++i) EXPECT_GT(log_times[i], log_times[i - 1]);
}

TEST(ZedOneFakeDeviceDeathTest, DestructorExitsWhenTheCaptureThreadIsStuckInGrab) {
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    EXPECT_EXIT(
        {
            auto state = std::make_shared<OneState>();
            state->block_grab = true;
            ZedOneRgbCameraConfiguration config = make_config(false);
            config.capture_thread_join_timeout_s = 0.3;
            auto camera = std::make_unique<ZedOneRgbCamera>(
                config, nullptr, std::make_unique<FakeZedOneDevice>(state), nullptr);
            camera->initialize();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            camera.reset();
            std::_Exit(0);
        },
        ::testing::ExitedWithCode(EXIT_FAILURE), "");
}

}  // namespace
}  // namespace auto_battlebot
#endif
