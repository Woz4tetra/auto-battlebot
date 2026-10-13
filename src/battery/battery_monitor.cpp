#include "battery/battery_monitor.hpp"

#include <spdlog/spdlog.h>

#include "battery/waveshare_ups.hpp"
#include "remote/status_bus.hpp"

namespace auto_battlebot {
namespace {

// Below this, the INA219 current is noise around zero and the battery is neither charging nor
// discharging in any way worth showing.
constexpr double kChargingCurrentA = 0.05;

}  // namespace

BatteryMonitor::BatteryMonitor(BatteryOptions options, SampleReader reader,
                               std::shared_ptr<remote::StatusBus> status_bus,
                               std::chrono::milliseconds period)
    : options_(std::move(options)),
      reader_(std::move(reader)),
      status_bus_(std::move(status_bus)),
      period_(period),
      estimator_(options_) {
    thread_ = std::thread([this] { run(); });
}

BatteryMonitor::~BatteryMonitor() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

std::optional<remote::BatteryMessage> BatteryMonitor::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

void BatteryMonitor::poll_once() {
    BatterySample sample;
    const bool ok = reader_(sample);
    if (!ok) {
        sample.timestamp = std::chrono::steady_clock::now();
        sample.valid = false;
    }
    const BatteryReading reading = estimator_.update(sample);
    estimator_.log_diagnostics(sample, reading, options_, ok);

    remote::BatteryMessage message;
    message.valid = reading.valid;
    message.percent = reading.percent;
    if (ok) {
        message.bus_voltage_v = sample.bus_voltage_v;
        message.current_a = sample.current_a;
        const double discharge_a =
            options_.discharge_current_positive ? sample.current_a : -sample.current_a;
        message.charging = discharge_a < -kChargingCurrentA;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = message;
    }
    if (status_bus_) status_bus_->publish(message);
}

void BatteryMonitor::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
        lock.unlock();
        poll_once();
        lock.lock();
        cv_.wait_for(lock, period_, [this] { return stop_; });
    }
}

std::unique_ptr<BatteryMonitor> make_battery_monitor(
    const BatteryConfiguration& config, std::shared_ptr<remote::StatusBus> status_bus) {
    switch (config.source) {
        case BatterySource::NONE:
            return nullptr;
        case BatterySource::WAVESHARE_UPS: {
            const BatteryOptions options = config.options;
            spdlog::info("Battery: Waveshare UPS on /dev/i2c-{} at 0x{:02x}", options.i2c_bus,
                         options.i2c_address);
            return std::make_unique<BatteryMonitor>(
                options,
                [options](BatterySample& sample) {
                    return read_waveshare_ups_sample(options, sample);
                },
                std::move(status_bus));
        }
    }
    return nullptr;
}

}  // namespace auto_battlebot
