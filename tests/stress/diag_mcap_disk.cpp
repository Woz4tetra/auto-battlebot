// Probe 8: eMMC fsync stall under sustained MCAP-style write load.
//
// Writes synthetic 6 MB messages at the runner's rate to a configurable output
// directory. Each write and each fsync is its own stage. Reports per-write and
// per-fsync latencies and flags eMMC stalls (>250 ms).

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "diag_common.hpp"
#include "spdlog/spdlog.h"

namespace ab = auto_battlebot::diag;

namespace {

double free_gb(const std::string& dir) {
    struct statvfs s {};
    if (statvfs(dir.c_str(), &s) != 0) return -1.0;
    const double bytes = static_cast<double>(s.f_bavail) * static_cast<double>(s.f_frsize);
    return bytes / (1024.0 * 1024.0 * 1024.0);
}

}  // namespace

int main(int argc, char** argv) {
    ab::install_sigsegv_backtrace();
    ab::configure_logger("diag_mcap_disk");

    std::string out_dir = "/tmp/diag_mcap";
    int message_bytes = 6 * 1024 * 1024;
    int rate_hz = 30;
    double duration_s = 60.0;
    int watchdog_s = 5;
    bool fsync_each = false;

    CLI::App app{"eMMC fsync stall probe"};
    app.add_option("--out-dir", out_dir, "Output directory");
    app.add_option("--message-bytes", message_bytes, "Bytes per write");
    app.add_option("--rate-hz", rate_hz, "Writes per second");
    app.add_option("--duration-s", duration_s, "Total run time in seconds");
    app.add_option("--watchdog-s", watchdog_s, "Per-stage hang threshold (seconds)");
    app.add_flag("--fsync-each", fsync_each, "fsync after every write (worst case)");
    CLI11_PARSE(app, argc, argv);

    ab::StageWatchdog wd(std::chrono::seconds(watchdog_s), "diag_mcap_disk");

    DIAG_STAGE(wd, "create_out_dir");
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        spdlog::error("create_directories({}) failed: {}", out_dir, ec.message());
        return ab::kExitSetupFailure;
    }

    const std::string file_path = out_dir + "/diag_mcap_payload.bin";
    DIAG_STAGE(wd, "open_output");
    const int fd = ::open(file_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        spdlog::error("open({}) failed errno={}", file_path, errno);
        return ab::kExitSetupFailure;
    }

    std::vector<uint8_t> payload(message_bytes);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(i & 0xff);

    const auto period = std::chrono::microseconds(1'000'000 / std::max(1, rate_hz));
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0));
    int writes = 0;
    int fsync_high = 0;
    int write_high = 0;
    char stage_buf[64];

    auto next = std::chrono::steady_clock::now();
    int i = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        std::snprintf(stage_buf, sizeof(stage_buf), "write_%d", i);
        wd.stage(stage_buf);
        const auto t0 = std::chrono::steady_clock::now();
        ssize_t n = ::write(fd, payload.data(), payload.size());
        const double w_ms = ab::elapsed_ms_since(t0);
        if (n != static_cast<ssize_t>(payload.size())) {
            spdlog::error("partial write n={} errno={}", n, errno);
            ::close(fd);
            return ab::kExitErrors;
        }
        if (w_ms > 250.0) {
            ++write_high;
            spdlog::warn("EMMC_LATENCY_HIGH op=write iter={} ms={:.2f}", i, w_ms);
        }

        if (fsync_each) {
            std::snprintf(stage_buf, sizeof(stage_buf), "fsync_%d", i);
            wd.stage(stage_buf);
            const auto t1 = std::chrono::steady_clock::now();
            if (fsync(fd) != 0) {
                spdlog::error("fsync failed errno={}", errno);
                ::close(fd);
                return ab::kExitErrors;
            }
            const double f_ms = ab::elapsed_ms_since(t1);
            if (f_ms > 250.0) {
                ++fsync_high;
                spdlog::warn("EMMC_LATENCY_HIGH op=fsync iter={} ms={:.2f}", i, f_ms);
            }
        }

        ++writes;
        if ((i + 1) % 30 == 0) {
            spdlog::info("writes={} free_gb={:.2f}", writes, free_gb(out_dir));
        }
        ++i;
        next += period;
        std::this_thread::sleep_until(next);
    }

    wd.stage("final_fsync");
    fsync(fd);
    wd.stage("close");
    ::close(fd);
    std::filesystem::remove(file_path, ec);

    spdlog::info("DONE writes={} write_high={} fsync_high={}", writes, write_high, fsync_high);
    if (write_high > 0 || fsync_high > 0) return ab::kExitErrors;
    return ab::kExitPass;
}
