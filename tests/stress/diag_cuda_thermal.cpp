// Probe 3: GPU thermal/power throttle detection.
//
// Sustained GPU work via cudaMemset on a large device buffer (kernel-backed),
// while polling Jetson sysfs for GPU/CPU frequency and thermal zones. Reports
// throttle events (frequency drops while temperature is high) and per-iter
// latency variance.

#include <cuda_runtime.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "diag_common.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

namespace {

std::string read_file_first_line(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string s;
    std::getline(f, s);
    return s;
}

struct ThermalZone {
    std::string type;
    std::string temp_path;
};

std::vector<ThermalZone> discover_thermal_zones() {
    std::vector<ThermalZone> zones;
    const std::filesystem::path root = "/sys/devices/virtual/thermal";
    if (!std::filesystem::exists(root)) return zones;
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        const auto name = entry.path().filename().string();
        if (name.rfind("thermal_zone", 0) != 0) continue;
        ThermalZone z;
        z.type = read_file_first_line((entry.path() / "type").string());
        z.temp_path = (entry.path() / "temp").string();
        zones.push_back(z);
    }
    return zones;
}

double read_temp_c(const std::string& path) {
    const std::string s = read_file_first_line(path);
    if (s.empty()) return -1.0;
    try {
        return std::stod(s) / 1000.0;
    } catch (...) {
        return -1.0;
    }
}

std::string read_gpu_freq_hz() {
    static const std::vector<std::string> candidates = {
        "/sys/class/devfreq/17000000.gv11b/cur_freq",
        "/sys/class/devfreq/17000000.ga10b/cur_freq",
        "/sys/devices/gpu.0/devfreq/17000000.gv11b/cur_freq",
        "/sys/devices/gpu.0/devfreq/17000000.ga10b/cur_freq",
    };
    for (const auto& p : candidates) {
        const std::string s = read_file_first_line(p);
        if (!s.empty()) return s;
    }
    return {};
}

}  // namespace

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_cuda_thermal");

    double duration_s = 60.0;
    int watchdog_s = 4;
    double temp_threshold_c = 80.0;
    size_t buffer_mb = 256;

    CLI::App app{"GPU thermal/throttle probe"};
    app.add_option("--duration-s", duration_s, "Total run time in seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_option("--temp-threshold-c", temp_threshold_c, "Warn above this temperature");
    app.add_option("--buffer-mb", buffer_mb, "Device buffer size used for cudaMemset");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_cuda_thermal");

    DIAG_STAGE(wd, "cudaMalloc");
    void* d_buf = nullptr;
    const size_t bytes = buffer_mb * 1024 * 1024;
    if (cudaMalloc(&d_buf, bytes) != cudaSuccess) {
        spdlog::error("cudaMalloc {} MB failed", buffer_mb);
        return ab::kExitSetupFailure;
    }

    const auto zones = discover_thermal_zones();
    spdlog::info("discovered {} thermal zones", zones.size());
    for (const auto& z : zones) spdlog::info("  zone type={} path={}", z.type, z.temp_path);

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    auto next_report = std::chrono::steady_clock::now();

    std::vector<double> iter_ms;
    iter_ms.reserve(8192);
    double baseline_ms = -1.0;
    int throttle_events = 0;
    int iter = 0;
    char stage_buf[64];
    std::string prev_freq;

    while (std::chrono::steady_clock::now() < deadline) {
        std::snprintf(stage_buf, sizeof(stage_buf), "memset_iter_%d", iter);
        wd.stage(stage_buf);
        const auto t0 = std::chrono::steady_clock::now();
        cudaMemset(d_buf, static_cast<int>(iter & 0xff), bytes);
        cudaDeviceSynchronize();
        const double ms = ab::elapsed_ms_since(t0);
        iter_ms.push_back(ms);

        if (baseline_ms < 0 && iter == 32) {
            // Use median of first 32 iters as baseline (warm-up done).
            std::vector<double> warm(iter_ms.begin(), iter_ms.end());
            std::sort(warm.begin(), warm.end());
            baseline_ms = warm[warm.size() / 2];
            spdlog::info("baseline_ms={:.3f}", baseline_ms);
        }

        if (std::chrono::steady_clock::now() >= next_report) {
            next_report += std::chrono::milliseconds(250);
            const std::string freq = read_gpu_freq_hz();
            double max_temp = -1.0;
            std::string hot_zone;
            for (const auto& z : zones) {
                const double t = read_temp_c(z.temp_path);
                if (t > max_temp) {
                    max_temp = t;
                    hot_zone = z.type;
                }
            }
            spdlog::info("t_ms={:.3f} gpu_freq_hz={} hottest_zone={} temp_c={:.1f}", ms, freq,
                         hot_zone, max_temp);
            if (!prev_freq.empty() && !freq.empty() && freq != prev_freq && max_temp > temp_threshold_c) {
                ++throttle_events;
                spdlog::warn("THROTTLE_DETECTED freq_prev={} freq_now={} temp_c={:.1f}", prev_freq,
                             freq, max_temp);
            }
            prev_freq = freq;
        }
        ++iter;
    }

    wd.stage("teardown");
    cudaFree(d_buf);

    if (iter_ms.empty()) return ab::kExitSetupFailure;
    std::vector<double> sorted = iter_ms;
    std::sort(sorted.begin(), sorted.end());
    const double max_ms = sorted.back();
    const double median_ms = sorted[sorted.size() / 2];
    spdlog::info("DONE iters={} median_ms={:.3f} max_ms={:.3f} throttle_events={}", iter_ms.size(),
                 median_ms, max_ms, throttle_events);

    if (baseline_ms > 0 && max_ms > 10.0 * baseline_ms) {
        spdlog::warn("CLASSIFICATION: latency_variance baseline_ms={:.3f} max_ms={:.3f}",
                     baseline_ms, max_ms);
        return ab::kExitErrors;
    }
    if (throttle_events > 0) {
        spdlog::warn("CLASSIFICATION: thermal_throttle events={}", throttle_events);
        return ab::kExitErrors;
    }
    return ab::kExitPass;
}
