// Probe 5: tegrastats popen pipe stall.
//
// Mirrors the popen + fgets pattern in src/health/health_logger.cpp.
// Wraps each fgets call as its own stage; if the pipe stalls, the watchdog
// names the iteration that hung.

#include <fcntl.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>

#include "diag_common.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_tegrastats");

    double duration_s = 60.0;
    int watchdog_s = 8;
    int interval_ms = 500;
    bool nonblocking_pipe = false;

    CLI::App app{"tegrastats popen/fgets stall probe"};
    app.add_option("--duration-s", duration_s, "Total run time in seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_option("--interval-ms", interval_ms, "tegrastats --interval value");
    app.add_flag("--nonblocking-pipe", nonblocking_pipe, "Set O_NONBLOCK on the pipe fd");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_tegrastats");

    DIAG_STAGE(wd, "popen");
    char cmd[128];
    std::snprintf(cmd, sizeof(cmd), "tegrastats --interval %d 2>&1", interval_ms);
    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        spdlog::error("popen failed errno={}", errno);
        return ab::kExitSetupFailure;
    }

    if (nonblocking_pipe) {
        const int fd = fileno(pipe);
        const int flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    char line[2048];
    char stage_buf[64];
    int lines = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        std::snprintf(stage_buf, sizeof(stage_buf), "fgets_iter_%d", lines);
        wd.stage(stage_buf);
        const auto t0 = std::chrono::steady_clock::now();
        if (fgets(line, sizeof(line), pipe) == nullptr) {
            if (feof(pipe)) {
                spdlog::warn("tegrastats EOF after {} lines", lines);
                break;
            }
            // Non-blocking pipe with no data ready.
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        const double ms = ab::elapsed_ms_since(t0);
        ++lines;
        if (ms > static_cast<double>(interval_ms) * 2.0) {
            spdlog::warn("SLOW_FGETS line={} elapsed_ms={:.2f}", lines, ms);
        }
        if (lines % 10 == 0) {
            spdlog::info("lines={} last_ms={:.2f}", lines, ms);
        }
    }

    wd.stage("pclose");
    pclose(pipe);

    spdlog::info("DONE lines={}", lines);
    if (lines == 0) {
        spdlog::warn("CLASSIFICATION: no_lines_received");
        return ab::kExitErrors;
    }
    return ab::kExitPass;
}
