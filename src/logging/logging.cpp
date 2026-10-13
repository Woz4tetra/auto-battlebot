#include "logging/logging.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
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
    explicit McapLogSink(std::shared_ptr<McapRecorder> recorder) : recorder_(std::move(recorder)) {
        rebuild_channel(nullptr);
    }

    void set_viz_sink(std::shared_ptr<VizSink> sink) {
        std::lock_guard<std::mutex> lock(mutex_);
        rebuild_channel(std::move(sink));
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

    void flush_() override {}

   private:
    void rebuild_channel(std::shared_ptr<VizSink> sink) {
        if (!recorder_ && !sink) {
            channel_.reset();
            return;
        }
        channel_ = std::make_unique<OutputChannel>(
            "/log", "protobuf", VizSchema::from_sdk(foxglove::schemas::Log::schema()), false,
            std::move(sink), recorder_);
    }

    std::shared_ptr<McapRecorder> recorder_;
    std::unique_ptr<OutputChannel> channel_;
};

McapLogSink* g_mcap_log_sink = nullptr;

}  // namespace

namespace {
constexpr const char* kLoggerName = "auto_battlebot";
constexpr const char* kPattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
/** Queued log lines. At the overrun policy below, a burst past this drops the oldest lines. */
constexpr size_t kLogQueueSize = 8192;
}  // namespace

void setup_logging(std::shared_ptr<McapRecorder> recorder) {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    auto mcap_sink = std::make_shared<McapLogSink>(std::move(recorder));
    g_mcap_log_sink = mcap_sink.get();
    sinks.push_back(std::move(mcap_sink));

    // Asynchronous: a log call formats the line, queues it and returns, and one spdlog thread
    // runs the sinks. The caller never waits on stdout, the viz socket or the recorder, so the
    // control thread can log mid-cycle. A full queue overwrites the oldest line rather than
    // blocking the caller.
    if (!spdlog::thread_pool()) spdlog::init_thread_pool(kLogQueueSize, 1);
    auto logger = std::make_shared<spdlog::async_logger>(
        kLoggerName, sinks.begin(), sinks.end(), spdlog::thread_pool(),
        spdlog::async_overflow_policy::overrun_oldest);
    logger->set_level(spdlog::level::debug);
    logger->set_pattern(kPattern);

    spdlog::set_default_logger(logger);
}

void shutdown_logging() {
    spdlog::default_logger()->flush();
    // Back to a synchronous console logger for whatever logs during static destruction. Dropping
    // the async logger also releases the recorder and viz sink it held, here rather than in an
    // unordered static teardown.
    g_mcap_log_sink = nullptr;
    auto logger = std::make_shared<spdlog::logger>(
        kLoggerName, std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    logger->set_level(spdlog::level::debug);
    logger->set_pattern(kPattern);
    spdlog::set_default_logger(logger);
}

void attach_log_viz_sink(std::shared_ptr<VizSink> sink) {
    if (!g_mcap_log_sink) return;
    g_mcap_log_sink->set_viz_sink(std::move(sink));
}

}  // namespace auto_battlebot
