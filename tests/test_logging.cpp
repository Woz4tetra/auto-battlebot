#include <gtest/gtest.h>
#include <spdlog/sinks/base_sink.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>

#include "logging/logging.hpp"

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

}  // namespace auto_battlebot
