#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "esp32_diagnostics/esp32_diagnostics_event.hpp"

namespace auto_battlebot {

struct Esp32WifiDiagnosticsOptions {
    /** The robot's access point address. An IP literal: no DNS lookup happens here. */
    std::string host = "192.168.4.1";
    int port = 80;
    /** Send `GET /record/start` before each stream so the firmware sends every control loop
     *  instead of at 10 Hz, and `GET /record/stop` on shutdown. */
    bool record_mode = true;
    /** First reconnect delay. Consecutive failures double it up to kMaxReconnectPeriod. */
    std::chrono::milliseconds reconnect_period{1000};
    /** A stream with no bytes for this long is treated as dropped. The firmware sends at 10 Hz
     *  even outside record mode, so silence means the robot is gone. */
    std::chrono::milliseconds stream_idle_timeout{3000};
    /** Parsed events held for drain(). Past this the oldest are dropped and counted. */
    size_t max_queued_events = 20000;
};

/** Cumulative counters since start(). */
struct Esp32WifiDiagnosticsStats {
    bool connected = false;
    uint64_t events = 0;
    uint64_t parse_errors = 0;
    /** Streams opened after the first. */
    uint64_t reconnects = 0;
    uint64_t connect_failures = 0;
    /** Events discarded because the queue was full. */
    uint64_t dropped_events = 0;
};

/**
 * Client for the Mr Stabs Mk2 firmware's diagnostics server over the robot's WiFi.
 *
 * A worker thread owns the socket: it sends `GET /record/start` when record mode is on, holds
 * the server-sent-events stream at `GET /events` open, parses every `data:` line into an
 * Esp32DiagnosticsEvent stamped with the host arrival time, and queues it. The caller drains
 * the queue; nothing here ever blocks the caller on the network. A dropped stream reconnects
 * with doubling backoff, re-sending `/record/start` since a rebooted robot comes back at 10 Hz.
 * stop() sends `GET /record/stop` from the worker before it exits.
 */
class Esp32WifiDiagnostics {
   public:
    using ClockNs = std::function<uint64_t()>;

    explicit Esp32WifiDiagnostics(Esp32WifiDiagnosticsOptions options, ClockNs clock_ns = {});
    ~Esp32WifiDiagnostics();

    Esp32WifiDiagnostics(const Esp32WifiDiagnostics &) = delete;
    Esp32WifiDiagnostics &operator=(const Esp32WifiDiagnostics &) = delete;

    /** Start the worker. Idempotent. */
    void start();
    /** Stop the worker and join it. Bounded by the socket timeouts, about a second. */
    void stop();

    /** Move every queued event out, oldest first. */
    std::vector<Esp32DiagnosticsEvent> drain();

    Esp32WifiDiagnosticsStats stats() const;

   private:
    void run();
    /** One connection cycle: record/start, then the event stream until it drops. Returns true
     *  when a stream was opened. */
    bool run_stream();
    /** `GET path` on a fresh connection; true on a 2xx status. */
    bool simple_get(const std::string &path, std::chrono::milliseconds timeout);
    int connect_socket(std::chrono::milliseconds timeout);
    bool send_request(int fd, const std::string &path, bool event_stream);
    void handle_sse_line(std::string_view line, std::string &event_name, std::string &data);
    void dispatch_event(const std::string &event_name, const std::string &data);
    void wait_for(std::chrono::milliseconds duration);
    /** True once stop() was called, except while the worker sends its final /record/stop. */
    bool stopping() const { return stop_.load() && !finishing_; }

    Esp32WifiDiagnosticsOptions options_;
    ClockNs clock_ns_;

    mutable std::mutex mutex_;
    std::condition_variable stop_cv_;
    std::deque<Esp32DiagnosticsEvent> queue_;
    Esp32WifiDiagnosticsStats stats_;
    bool streamed_before_ = false;

    std::atomic<bool> stop_{false};
    /** Worker thread only. */
    bool finishing_ = false;
    std::thread thread_;
};

}  // namespace auto_battlebot
