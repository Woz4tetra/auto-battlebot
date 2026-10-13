#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "battery/battery_soc_estimator.hpp"
#include "battery/config.hpp"
#include "remote/messages.hpp"

namespace auto_battlebot {

namespace remote {
class StatusBus;
}

/**
 * Reads the robot computer's battery on its own thread and publishes /status/battery.
 *
 * Each period it takes one sample, steps the estimator, logs the `battery_ina219` diagnostics,
 * and publishes. An I2C read can take tens of milliseconds, which is why this never runs on the
 * main loop. Construct it after DiagnosticsLogger::initialize, or the diagnostics are dropped.
 */
class BatteryMonitor {
   public:
    /** Fills `sample` and returns true, or returns false when the read failed. Replaced in tests.
     */
    using SampleReader = std::function<bool(BatterySample& sample)>;

    BatteryMonitor(BatteryOptions options, SampleReader reader,
                   std::shared_ptr<remote::StatusBus> status_bus,
                   std::chrono::milliseconds period = std::chrono::milliseconds(1000));
    ~BatteryMonitor();

    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

    /** The last message published; empty before the first read. */
    std::optional<remote::BatteryMessage> latest() const;

   private:
    void run();
    void poll_once();

    BatteryOptions options_;
    SampleReader reader_;
    std::shared_ptr<remote::StatusBus> status_bus_;
    std::chrono::milliseconds period_;
    BatterySocEstimator estimator_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
    std::optional<remote::BatteryMessage> latest_;
    std::thread thread_;
};

/** A monitor for `config.source`, or nullptr for NONE. */
std::unique_ptr<BatteryMonitor> make_battery_monitor(const BatteryConfiguration& config,
                                                     std::shared_ptr<remote::StatusBus> status_bus);

}  // namespace auto_battlebot
