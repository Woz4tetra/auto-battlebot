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
    /** The robot's address, an IP literal: no DNS lookup happens here. */
    std::string host = "192.168.4.1";
    int port = 80;
    /** Time between polls while caught up. The ring holds two minutes, so this only sets how
     *  stale the newest row is when it lands, not whether rows are lost. */
    std::chrono::milliseconds poll_period{100};
    /** Wait after a failed poll before the next one. */
    std::chrono::milliseconds retry_period{1000};
};

struct Esp32WifiTelemetryStats {
    /** The last poll got a valid reply. */
    bool connected = false;
    uint64_t polls = 0;
    /** Polls that timed out, were refused, or answered non-2xx. */
    uint64_t poll_failures = 0;
    /** Replies that did not decode: wrong magic, version, record size, or length. */
    uint64_t parse_errors = 0;
    uint64_t events = 0;
    /** Events marked backfill: everything except the newest row of a poll that caught up. */
    uint64_t backfilled = 0;
    /** Records overwritten in the ring before a poll reached them. */
    uint64_t lost_records = 0;
    /** Times the robot's boot id changed, so seqs restarted. */
    uint64_t reboots = 0;
    /** Events discarded because the queue was full (the caller stopped draining). */
    uint64_t dropped_events = 0;
    /** Records still waiting in the ring after the last poll. */
    uint32_t lag_records = 0;
    uint32_t ring_capacity = 0;
};

/**
 * Reads the Mr Stabs Mk2 firmware's telemetry ring over WiFi (`GET /telemetry`).
 *
 * A worker thread polls with the seq after the last record it holds, so a reply carries every
 * record since, and a link that drops costs time but no rows while the ring still holds them. The
 * first poll after start or a robot reboot only learns where the ring's head is: history from
 * before this client was listening is not replayed. Each record becomes an Esp32DiagnosticsEvent
 * stamped with the reply's arrival time. All but the newest row of a poll that caught up are
 * marked backfill, since their arrival time says when the poll ran, not when the robot sent them.
 * Nothing here blocks the caller on the network.
 */
class Esp32WifiTelemetry {
   public:
    using ClockNs = std::function<uint64_t()>;

    explicit Esp32WifiTelemetry(Esp32WifiTelemetryOptions options, ClockNs clock_ns = {});
    ~Esp32WifiTelemetry();
    Esp32WifiTelemetry(const Esp32WifiTelemetry &) = delete;
    Esp32WifiTelemetry &operator=(const Esp32WifiTelemetry &) = delete;

    /** Start the worker. Idempotent. */
    void start();
    /** Stop the worker and join it. Bounded by the request timeout, about a second. */
    void stop();
    /** Move every queued event out, oldest first. */
    std::vector<Esp32DiagnosticsEvent> drain();
    Esp32WifiTelemetryStats stats() const;

   private:
    void run();
    /** Returns false when the poll failed and the worker should back off. Sets `more` when the
     *  ring still holds records past this reply. */
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

    // Worker-only state.
    std::optional<uint32_t> boot_id_;
    uint32_t next_seq_ = 0;
};

}  // namespace auto_battlebot
