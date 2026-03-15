#include "logging/logging.hpp"

// Undefine macros that conflict with rosgraph_msgs/Log.hxx enum constants
#ifdef DEBUG
#undef DEBUG
#endif
#ifdef ERROR
#undef ERROR
#endif
#ifdef INFO
#undef INFO
#endif
#ifdef WARN
#undef WARN
#endif
#ifdef FATAL
#undef FATAL
#endif

#include <miniros/node_handle.h>
#include <miniros/rostime.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <rosgraph_msgs/Log.hxx>
#include <vector>

#include "mcap_recorder/mcap_recorder.hpp"

namespace auto_battlebot {

namespace {

class McapLogSink : public spdlog::sinks::base_sink<std::mutex> {
   public:
    explicit McapLogSink(std::shared_ptr<McapRecorder> recorder) : recorder_(std::move(recorder)) {}

    void set_publisher(std::shared_ptr<miniros::Publisher> pub) {
        std::lock_guard<std::mutex> lock(mutex_);
        rosout_pub_ = std::move(pub);
    }

   protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        rosgraph_msgs::Log log_msg;
        double stamp_secs = std::chrono::duration<double>(msg.time.time_since_epoch()).count();
        log_msg.header.stamp = miniros::Time(stamp_secs);

        switch (msg.level) {
            case spdlog::level::trace:
            case spdlog::level::debug:
                log_msg.level = rosgraph_msgs::Log::DEBUG;
                break;
            case spdlog::level::info:
                log_msg.level = rosgraph_msgs::Log::INFO;
                break;
            case spdlog::level::warn:
                log_msg.level = rosgraph_msgs::Log::WARN;
                break;
            case spdlog::level::err:
                log_msg.level = rosgraph_msgs::Log::ERROR;
                break;
            case spdlog::level::critical:
                log_msg.level = rosgraph_msgs::Log::FATAL;
                break;
            default:
                log_msg.level = rosgraph_msgs::Log::INFO;
                break;
        }

        log_msg.name = std::string(msg.logger_name.data(), msg.logger_name.size());
        log_msg.msg = std::string(msg.payload.data(), msg.payload.size());
        log_msg.file = msg.source.filename ? msg.source.filename : "";
        log_msg.function = msg.source.funcname ? msg.source.funcname : "";
        log_msg.line = static_cast<uint32_t>(msg.source.line);

        uint64_t time_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(msg.time.time_since_epoch())
                .count());

        if (recorder_) {
            recorder_->write("/rosout", log_msg, time_ns);
        }

        if (rosout_pub_) {
            rosout_pub_->publish(log_msg);
        }
    }

    void flush_() override {}

   private:
    std::shared_ptr<McapRecorder> recorder_;
    std::shared_ptr<miniros::Publisher> rosout_pub_;
};

McapLogSink* g_mcap_log_sink = nullptr;

}  // namespace

void setup_logging(std::shared_ptr<McapRecorder> recorder) {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    auto mcap_sink = std::make_shared<McapLogSink>(std::move(recorder));
    g_mcap_log_sink = mcap_sink.get();
    sinks.push_back(std::move(mcap_sink));

    auto logger = std::make_shared<spdlog::logger>("auto_battlebot", sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::debug);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    spdlog::set_default_logger(logger);
}

void setup_rosout_publisher(miniros::NodeHandle& nh) {
    if (!g_mcap_log_sink) return;
    auto pub =
        std::make_shared<miniros::Publisher>(nh.advertise<rosgraph_msgs::Log>("/rosout", 100));
    g_mcap_log_sink->set_publisher(std::move(pub));
}

}  // namespace auto_battlebot
