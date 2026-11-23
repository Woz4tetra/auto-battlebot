#include "logging/logging.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <foxglove/schemas.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

namespace {

foxglove::schemas::Log::LogLevel to_log_level(spdlog::level::level_enum level) {
    using Level = foxglove::schemas::Log::LogLevel;
    switch (level) {
        case spdlog::level::trace:
        case spdlog::level::debug:
            return Level::DEBUG;
        case spdlog::level::info:
            return Level::INFO;
        case spdlog::level::warn:
            return Level::WARNING;
        case spdlog::level::err:
            return Level::ERROR;
        case spdlog::level::critical:
            return Level::FATAL;
        default:
            return Level::INFO;
    }
}

class McapLogSink : public spdlog::sinks::base_sink<std::mutex> {
   public:
    McapLogSink() = default;

    void set_recorder(std::shared_ptr<McapRecorder> recorder) {
        std::lock_guard<std::mutex> lock(mutex_);
        recorder_ = std::move(recorder);
        rebuild_channel(viz_sink_);
    }

    void set_viz_sink(std::shared_ptr<VizSink> sink) {
        std::lock_guard<std::mutex> lock(mutex_);
        viz_sink_ = std::move(sink);
        rebuild_channel(viz_sink_);
    }

    bool flush_logger_and_wait(spdlog::logger& logger, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(flush_mutex_);
        const uint64_t before = flushes_;
        lock.unlock();
        logger.flush();
        lock.lock();
        return flushed_cv_.wait_for(lock, timeout, [&] { return flushes_ > before; });
    }

   protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        if (!channel_) return;
        foxglove::schemas::Log log;
        const auto since_epoch = msg.time.time_since_epoch();
        const auto secs = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
        foxglove::schemas::Timestamp stamp;
        stamp.sec = static_cast<uint32_t>(secs.count());
        stamp.nsec = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs).count());
        log.timestamp = stamp;
        log.level = to_log_level(msg.level);
        log.name = std::string(msg.logger_name.data(), msg.logger_name.size());
        log.message = std::string(msg.payload.data(), msg.payload.size());
        log.file = msg.source.filename ? msg.source.filename : "";
        log.line = static_cast<uint32_t>(msg.source.line);

        const auto time_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch).count());
        channel_->log_message(log, time_ns);
    }

    void flush_() override {
        {
            std::lock_guard<std::mutex> lock(flush_mutex_);
            ++flushes_;
        }
        flushed_cv_.notify_all();
    }

   private:
    void rebuild_channel(const std::shared_ptr<VizSink>& sink) {
        if (!recorder_ && !sink) {
            channel_.reset();
            return;
        }
        channel_ = std::make_unique<OutputChannel>(
            viz_topic(VizTopicId::LOG), VizSchema::from_sdk(foxglove::schemas::Log::schema()), sink,
            recorder_);
    }

    std::shared_ptr<McapRecorder> recorder_;
    std::shared_ptr<VizSink> viz_sink_;
    std::unique_ptr<OutputChannel> channel_;
    std::mutex flush_mutex_;
    uint64_t flushes_ = 0;
    std::condition_variable flushed_cv_;
};

std::mutex g_mcap_log_sink_mutex;
std::weak_ptr<McapLogSink> g_mcap_log_sink;
std::weak_ptr<spdlog::logger> g_mcap_logger;

}  // namespace

namespace {
constexpr const char* kLoggerName = "auto_battlebot";
constexpr const char* kPattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
constexpr size_t kLogQueueSize = 8192;
}  // namespace

std::shared_ptr<spdlog::logger> make_async_logger(
    const std::vector<spdlog::sink_ptr>& sinks,
    std::shared_ptr<spdlog::details::thread_pool> pool) {
    return std::make_shared<spdlog::async_logger>(kLoggerName, sinks.begin(), sinks.end(),
                                                  std::move(pool),
                                                  spdlog::async_overflow_policy::overrun_oldest);
}

void setup_logging() {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    auto mcap_sink = std::make_shared<McapLogSink>();
    {
        std::lock_guard<std::mutex> lock(g_mcap_log_sink_mutex);
        g_mcap_log_sink = mcap_sink;
    }
    sinks.push_back(std::move(mcap_sink));

    if (!spdlog::thread_pool()) spdlog::init_thread_pool(kLogQueueSize, 1);
    auto logger = make_async_logger(sinks, spdlog::thread_pool());
    logger->set_level(spdlog::level::debug);
    logger->set_pattern(kPattern);

    {
        std::lock_guard<std::mutex> lock(g_mcap_log_sink_mutex);
        g_mcap_logger = logger;
    }
    spdlog::set_default_logger(logger);
}

void attach_log_recorder(std::shared_ptr<McapRecorder> recorder) {
    std::shared_ptr<McapLogSink> mcap_sink;
    {
        std::lock_guard<std::mutex> lock(g_mcap_log_sink_mutex);
        mcap_sink = g_mcap_log_sink.lock();
    }
    if (!mcap_sink) return;
    mcap_sink->set_recorder(std::move(recorder));
}

void shutdown_logging() {
    spdlog::default_logger()->flush();
    {
        std::lock_guard<std::mutex> lock(g_mcap_log_sink_mutex);
        g_mcap_log_sink.reset();
    }
    auto logger = std::make_shared<spdlog::logger>(
        kLoggerName, std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    logger->set_level(spdlog::level::debug);
    logger->set_pattern(kPattern);
    spdlog::set_default_logger(logger);
}

void attach_log_viz_sink(std::shared_ptr<VizSink> sink) {
    std::shared_ptr<McapLogSink> mcap_sink;
    {
        std::lock_guard<std::mutex> lock(g_mcap_log_sink_mutex);
        mcap_sink = g_mcap_log_sink.lock();
    }
    if (!mcap_sink) return;
    mcap_sink->set_viz_sink(std::move(sink));
}

void flush_logging(std::chrono::milliseconds timeout) {
    std::shared_ptr<McapLogSink> mcap_sink;
    std::shared_ptr<spdlog::logger> logger;
    {
        std::lock_guard<std::mutex> lock(g_mcap_log_sink_mutex);
        mcap_sink = g_mcap_log_sink.lock();
        logger = g_mcap_logger.lock();
    }
    if (!mcap_sink || !logger) return;
    mcap_sink->flush_logger_and_wait(*logger, timeout);
}

}  // namespace auto_battlebot
