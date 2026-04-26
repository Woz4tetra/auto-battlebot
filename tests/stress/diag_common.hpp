#pragma once

#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "spdlog/spdlog.h"

namespace auto_battlebot::diag {

constexpr int kExitPass = 0;
constexpr int kExitHang = 10;
constexpr int kExitErrors = 20;
constexpr int kExitInconclusive = 30;
constexpr int kExitSetupFailure = 40;

inline int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

inline double elapsed_ms_since(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

class StageWatchdog {
   public:
    StageWatchdog(std::chrono::seconds deadline, const char* probe_name)
        : deadline_ns_(static_cast<int64_t>(deadline.count()) * 1'000'000'000LL),
          probe_name_(probe_name),
          last_stage_("init"),
          last_update_ns_(now_ns()),
          stop_(false) {
        thread_ = std::thread([this]() { this->loop(); });
    }

    ~StageWatchdog() {
        stop_.store(true);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    StageWatchdog(const StageWatchdog&) = delete;
    StageWatchdog& operator=(const StageWatchdog&) = delete;

    void stage(const char* name) {
        last_stage_.store(name, std::memory_order_release);
        last_update_ns_.store(now_ns(), std::memory_order_release);
    }

    const char* current_stage() const { return last_stage_.load(std::memory_order_acquire); }

   private:
    void loop() {
        while (!stop_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const int64_t since = now_ns() - last_update_ns_.load(std::memory_order_acquire);
            if (since > deadline_ns_) {
                fire();
                return;
            }
        }
    }

    [[noreturn]] void fire() {
        const char* stage = last_stage_.load(std::memory_order_acquire);
        char buf[256];
        const int n = std::snprintf(buf, sizeof(buf),
                                    "STUCK AT: %s (probe=%s, deadline_s=%.1f)\n",
                                    stage ? stage : "<null>", probe_name_,
                                    static_cast<double>(deadline_ns_) / 1e9);
        if (n > 0) {
            ssize_t ignored = write(STDERR_FILENO, buf, static_cast<size_t>(n));
            (void)ignored;
        }
        spdlog::shutdown();
        std::abort();
    }

    int64_t deadline_ns_;
    const char* probe_name_;
    std::atomic<const char*> last_stage_;
    std::atomic<int64_t> last_update_ns_;
    std::atomic<bool> stop_;
    std::thread thread_;
};

#define DIAG_STAGE(wd, name)                                                              \
    do {                                                                                  \
        (wd).stage(name);                                                                 \
        spdlog::info("ts_ns={} stage={}", ::auto_battlebot::diag::now_ns(), (name));      \
    } while (0)

inline void install_sigsegv_backtrace() {
    struct sigaction sa {};
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = [](int sig, siginfo_t*, void*) {
        void* frames[64];
        const int n = backtrace(frames, 64);
        const char* msg = "DIAG_SIGNAL: backtrace below\n";
        ssize_t ignored = write(STDERR_FILENO, msg, std::strlen(msg));
        (void)ignored;
        backtrace_symbols_fd(frames, n, STDERR_FILENO);
        signal(sig, SIG_DFL);
        raise(sig);
    };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGSEGV, &sa, nullptr);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGBUS, &sa, nullptr);
}

inline void configure_logger(const std::string& probe_name) {
    spdlog::set_pattern("[%H:%M:%S.%f] [" + probe_name + "] %v");
    spdlog::set_level(spdlog::level::info);
}

}  // namespace auto_battlebot::diag
