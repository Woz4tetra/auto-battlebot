// Probe 7: serial write blocking-under-back-pressure probe.
//
// Reuses SerialPort from src/serial/serial_port.cpp. Sends synthetic CRSF-sized
// frames at the configured rate. Each ::write call is its own stage. Optionally
// auto-detects the OpenTX VID/PID device.

#include <CLI/CLI.hpp>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "diag_common.hpp"
#include "serial/serial_port.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_serial_opentx");

    std::string device;
    int baud = 115200;
    int rate_hz = 50;
    int frame_bytes = 26;  // typical CRSF channel-bundle frame
    double duration_s = 60.0;
    int watchdog_s = 2;
    bool unplug_test = false;

    CLI::App app{"OpenTX serial write hang probe"};
    app.add_option("--device", device, "TTY path (default: auto-detect VID 0x0483 PID 0x5740)");
    app.add_option("--baud", baud, "Baud rate");
    app.add_option("--rate-hz", rate_hz, "Frames per second");
    app.add_option("--frame-bytes", frame_bytes, "Bytes per frame");
    app.add_option("--duration-s", duration_s, "Total run time in seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_flag("--unplug-test", unplug_test,
                 "Block waiting for the user to unplug the dongle, then verify writes fail "
                 "rather than block");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_serial_opentx");

    if (device.empty()) {
        DIAG_STAGE(wd, "find_opentx_device");
        const auto found = auto_battlebot::find_opentx_device();
        if (!found) {
            spdlog::error("could not auto-detect OpenTX device; pass --device");
            return ab::kExitSetupFailure;
        }
        device = *found;
        spdlog::info("auto-detected device: {}", device);
    }

    DIAG_STAGE(wd, "open");
    auto_battlebot::SerialPort port;
    if (!port.open(device, baud)) {
        spdlog::error("open({}) failed", device);
        return ab::kExitSetupFailure;
    }

    std::vector<uint8_t> frame(frame_bytes);
    for (int i = 0; i < frame_bytes; ++i) frame[i] = static_cast<uint8_t>(i);

    const auto period = std::chrono::microseconds(1'000'000 / std::max(1, rate_hz));
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    int writes_ok = 0;
    int writes_fail = 0;
    char stage_buf[64];

    if (unplug_test) {
        spdlog::warn("UNPLUG TEST: disconnect the dongle within 10 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    auto next = std::chrono::steady_clock::now();
    int i = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        std::snprintf(stage_buf, sizeof(stage_buf), "write_%d", i);
        wd.stage(stage_buf);
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = port.write(frame.data(), frame.size());
        const double ms = ab::elapsed_ms_since(t0);
        if (ok) {
            ++writes_ok;
        } else {
            ++writes_fail;
            if (unplug_test) {
                spdlog::info("write failed as expected after unplug: iter={} ms={:.2f}", i, ms);
                return ab::kExitPass;
            }
        }
        if (ms > 50.0) {
            spdlog::warn("SLOW_WRITE iter={} ms={:.2f}", i, ms);
        }
        ++i;
        next += period;
        std::this_thread::sleep_until(next);
    }

    wd.stage("close");
    port.close();

    spdlog::info("DONE writes_ok={} writes_fail={}", writes_ok, writes_fail);
    if (unplug_test) {
        spdlog::error("CLASSIFICATION: unplug_test_writes_kept_succeeding");
        return ab::kExitErrors;
    }
    if (writes_fail > 0 && writes_fail > writes_ok / 10) return ab::kExitErrors;
    return ab::kExitPass;
}
