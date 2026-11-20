#include <gtest/gtest.h>
#include <spdlog/sinks/base_sink.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>

#include "logging/logging.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "scratch_recordings.hpp"

namespace auto_battlebot {
namespace {

class GatedSink : public spdlog::sinks::base_sink<std::mutex> {
   public:
    void open() {
        {
            std::lock_guard<std::mutex> lock(gate_mutex_);
            open_ = true;
        }
        gate_.notify_all();
    }

   protected:
    void sink_it_(const spdlog::details::log_msg &) override {
        std::unique_lock<std::mutex> lock(gate_mutex_);
        gate_.wait(lock, [this] { return open_; });
    }
    void flush_() override {}

   private:
    std::mutex gate_mutex_;
    std::condition_variable gate_;
    bool open_ = false;
};

}  // namespace

TEST(LoggingTest, FullQueueDropsOldestWithoutBlocking) {
    auto sink = std::make_shared<GatedSink>();
    auto pool = std::make_shared<spdlog::details::thread_pool>(4, 1);
    auto logger = make_async_logger({sink}, pool);

    auto producer = std::async(std::launch::async, [&logger] {
        for (int i = 0; i < 100; ++i) {
            logger->info("line {}", i);
        }
    });
    const auto status = producer.wait_for(std::chrono::seconds(2));
    sink->open();
    EXPECT_EQ(status, std::future_status::ready) << "a stalled sink blocked the caller";
}

TEST(LoggingTest, ShutdownReleasesTheRecorderAndLeavesASynchronousLogger) {
    const auto previous = spdlog::default_logger();
    const auto dir = make_scratch_recordings_dir("logging_shutdown_test");
    auto recorder = std::make_shared<McapRecorder>("logging_shutdown_test", dir);
    std::weak_ptr<McapRecorder> watcher = recorder;

    setup_logging(recorder);
    recorder.reset();
    EXPECT_FALSE(watcher.expired()) << "the async logger should hold the recorder";
    EXPECT_NE(std::dynamic_pointer_cast<spdlog::async_logger>(spdlog::default_logger()), nullptr);

    shutdown_logging();
    EXPECT_TRUE(watcher.expired()) << "the recorder outlived shutdown_logging";
    EXPECT_EQ(std::dynamic_pointer_cast<spdlog::async_logger>(spdlog::default_logger()), nullptr);
    spdlog::info("logged after shutdown");

    spdlog::set_default_logger(previous);
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
}

}  // namespace auto_battlebot
