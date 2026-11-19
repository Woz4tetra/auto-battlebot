#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "esp32_diagnostics/esp32_diagnostics_event.hpp"

namespace auto_battlebot {

struct Esp32WifiTelemetryOptions {
    std::string host = "192.168.4.1";
    int port = 80;
    std::chrono::milliseconds poll_period{100};
    std::chrono::milliseconds retry_period{1000};
};

struct Esp32WifiTelemetryStats {
    bool connected = false;
    uint64_t polls = 0;
    uint64_t poll_failures = 0;
    uint64_t parse_errors = 0;
    uint64_t events = 0;
    uint64_t backfilled = 0;
    uint64_t ring_overwritten_records = 0;
    uint64_t boot_id_changes = 0;
    uint64_t dropped_events = 0;
    uint32_t lag_records = 0;
    uint32_t ring_capacity = 0;
};

class Esp32WifiTelemetry {
   public:
    using ClockNs = std::function<uint64_t()>;

    explicit Esp32WifiTelemetry(Esp32WifiTelemetryOptions options, ClockNs clock_ns = {});
    ~Esp32WifiTelemetry();
    Esp32WifiTelemetry(const Esp32WifiTelemetry &) = delete;
    Esp32WifiTelemetry &operator=(const Esp32WifiTelemetry &) = delete;

    void start();
    void stop();
    std::vector<Esp32DiagnosticsEvent> drain();
    Esp32WifiTelemetryStats stats() const;

   private:
    void run();
    bool poll_once(bool &more);
    std::optional<std::string> http_get(const std::string &path);
    void wait_for(std::chrono::milliseconds duration);

    Esp32WifiTelemetryOptions options_;
    ClockNs clock_ns_;

    std::thread thread_;
    std::atomic<bool> stop_{false};
    mutable std::mutex mutex_;
    std::condition_variable stop_cv_;
    std::deque<Esp32DiagnosticsEvent> queue_;
    Esp32WifiTelemetryStats stats_;

    std::optional<uint32_t> boot_id_;
    uint32_t next_seq_ = 0;
};

}  // namespace auto_battlebot
