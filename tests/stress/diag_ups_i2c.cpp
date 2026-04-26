// Probe 6: Waveshare UPS INA219 I2C blocking-read probe.
//
// Mirrors the read pattern in src/ui/lvgl_platform_bound/lvgl_ui_battery.cpp.
// Each read is staged separately so a hang names the register being read and
// the iteration. Optionally spawns a background thread doing tegrastats reads
// to stress bus arbitration.

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "diag_common.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

namespace {

constexpr uint8_t INA219_REG_CONFIG = 0x00;
constexpr uint8_t INA219_REG_BUSVOLTAGE = 0x02;
constexpr uint8_t INA219_REG_POWER = 0x03;
constexpr uint8_t INA219_REG_CURRENT = 0x04;
constexpr uint8_t INA219_REG_CALIBRATION = 0x05;
constexpr uint16_t INA219_CALIBRATION_16V_5A = 26868;
constexpr uint16_t INA219_CONFIG_16V_5A_CONTINUOUS = 0x0EEF;

bool write_u16(int fd, uint8_t reg, uint16_t value) {
    uint8_t payload[3] = {reg, static_cast<uint8_t>((value >> 8) & 0xFF),
                          static_cast<uint8_t>(value & 0xFF)};
    return write(fd, payload, sizeof(payload)) == static_cast<ssize_t>(sizeof(payload));
}

bool read_u16(int fd, uint8_t reg, uint16_t& value) {
    if (write(fd, &reg, sizeof(reg)) != static_cast<ssize_t>(sizeof(reg))) return false;
    uint8_t data[2] = {0, 0};
    if (read(fd, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data))) return false;
    value = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_ups_i2c");

    int bus = 1;
    int address = 0x42;
    int iterations = 0;
    double duration_s = 60.0;
    int watchdog_s = 3;
    bool with_bus_contention = false;

    CLI::App app{"Waveshare UPS / INA219 I2C blocking-read probe"};
    app.add_option("--bus", bus, "I2C bus number (e.g. 1 for /dev/i2c-1)");
    app.add_option("--address", address, "I2C device address (default 0x42)");
    app.add_option("--iterations", iterations, "Stop after N iterations (0 = use --duration-s)");
    app.add_option("--duration-s", duration_s, "Total run time in seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_flag("--with-bus-contention", with_bus_contention,
                 "Spawn background tegrastats popen reads to contend the bus");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_ups_i2c");

    std::atomic<bool> stop_contention{false};
    std::thread contention_thread;
    if (with_bus_contention) {
        contention_thread = std::thread([&stop_contention]() {
            while (!stop_contention.load(std::memory_order_relaxed)) {
                FILE* p = popen("tegrastats --interval 200 2>&1", "r");
                if (!p) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                char buf[1024];
                for (int i = 0; i < 5; ++i) {
                    if (fgets(buf, sizeof(buf), p) == nullptr) break;
                }
                pclose(p);
            }
        });
    }

    int slow_reads = 0;
    int errors = 0;
    int total = 0;

    const std::string dev_path = "/dev/i2c-" + std::to_string(bus);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    char stage_buf[64];

    for (int i = 0; iterations == 0 || i < iterations; ++i) {
        if (iterations == 0 && std::chrono::steady_clock::now() > deadline) break;

        std::snprintf(stage_buf, sizeof(stage_buf), "open_%d", i);
        wd.stage(stage_buf);
        int fd = open(dev_path.c_str(), O_RDWR);
        if (fd < 0) {
            ++errors;
            spdlog::warn("open {} failed errno={}", dev_path, errno);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::snprintf(stage_buf, sizeof(stage_buf), "ioctl_slave_%d", i);
        wd.stage(stage_buf);
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            ++errors;
            close(fd);
            continue;
        }

        bool ok = true;
        const auto t0 = std::chrono::steady_clock::now();

        std::snprintf(stage_buf, sizeof(stage_buf), "write_calibration_%d", i);
        wd.stage(stage_buf);
        ok &= write_u16(fd, INA219_REG_CALIBRATION, INA219_CALIBRATION_16V_5A);

        std::snprintf(stage_buf, sizeof(stage_buf), "write_config_%d", i);
        wd.stage(stage_buf);
        ok &= write_u16(fd, INA219_REG_CONFIG, INA219_CONFIG_16V_5A_CONTINUOUS);

        uint16_t v_bus = 0, v_cur = 0, v_pow = 0;
        std::snprintf(stage_buf, sizeof(stage_buf), "read_busvoltage_%d", i);
        wd.stage(stage_buf);
        ok &= read_u16(fd, INA219_REG_BUSVOLTAGE, v_bus);

        std::snprintf(stage_buf, sizeof(stage_buf), "read_current_%d", i);
        wd.stage(stage_buf);
        ok &= read_u16(fd, INA219_REG_CURRENT, v_cur);

        std::snprintf(stage_buf, sizeof(stage_buf), "read_power_%d", i);
        wd.stage(stage_buf);
        ok &= read_u16(fd, INA219_REG_POWER, v_pow);

        const double ms = ab::elapsed_ms_since(t0);
        close(fd);

        if (!ok) {
            ++errors;
        } else {
            ++total;
            if (ms > 100.0) {
                ++slow_reads;
                spdlog::warn("SLOW_I2C_READ ms={:.2f} iter={} bus={} v_cur={} v_pow={}", ms, i,
                             v_bus, v_cur, v_pow);
            }
            if (ms > 1000.0) {
                spdlog::error("CLASSIFICATION: i2c_read_over_1s ms={:.2f}", ms);
                stop_contention.store(true);
                if (contention_thread.joinable()) contention_thread.join();
                return ab::kExitHang;
            }
        }
    }

    stop_contention.store(true);
    if (contention_thread.joinable()) contention_thread.join();

    spdlog::info("DONE total={} errors={} slow_reads={}", total, errors, slow_reads);
    if (total == 0) return ab::kExitSetupFailure;
    if (slow_reads > 0) return ab::kExitErrors;
    return ab::kExitPass;
}
