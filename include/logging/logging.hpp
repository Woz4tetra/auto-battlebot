#pragma once

#include <spdlog/async.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <vector>

namespace auto_battlebot {

class McapRecorder;
class VizSink;

std::shared_ptr<spdlog::logger> make_async_logger(
    const std::vector<spdlog::sink_ptr>& sinks, std::shared_ptr<spdlog::details::thread_pool> pool);

void setup_logging(std::shared_ptr<McapRecorder> recorder);

void shutdown_logging();

void flush_logging(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

void attach_log_viz_sink(std::shared_ptr<VizSink> sink);

}  // namespace auto_battlebot
