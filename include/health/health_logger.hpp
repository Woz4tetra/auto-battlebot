#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "health/config.hpp"

namespace auto_battlebot {

class HealthLogger {
   public:
    explicit HealthLogger(const HealthConfiguration& config);
    void maybe_log();
    void record_tick(double tick_ms);

    /** Most-recently collected Jetson SoC temperature (degrees C). 0 = not yet available. */
    double get_last_temp_c() const { return last_temp_c_; }
    /** Most-recently collected nvpmodel compute mode string (e.g. "MAXN"). Empty = not available. */
    const std::string& get_last_compute_mode() const { return last_compute_mode_; }

   private:
    HealthConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::chrono::steady_clock::time_point last_sample_time_;
    bool has_sampled_ = false;
    std::shared_ptr<FILE> tegrastats_pipe_;
    bool tegrastats_unavailable_ = false;

    double last_temp_c_ = 0.0;
    std::string last_compute_mode_;

    std::chrono::steady_clock::time_point heartbeat_window_start_;
    bool heartbeat_window_started_ = false;
    uint64_t heartbeat_ticks_ = 0;
    double max_tick_ms_ = 0.0;

    void perform_sampling();
    void emit_heartbeat_if_due(std::chrono::steady_clock::time_point now);

    static bool is_x86();
    static bool command_exists(const std::string& command);
    static std::string trim(const std::string& value);
    static std::vector<std::string> split(const std::string& input, char delimiter);
    static bool run_command(const std::string& command, std::string& output);
    static bool parse_double(const std::string& value, double& out);
    static bool parse_int(const std::string& value, int& out);

    bool start_tegrastats_stream();
    void stop_tegrastats_stream();
    bool collect_tegrastats();
    void collect_compute_mode();
    bool collect_x86_health();
    bool collect_nvidia_smi();
    bool collect_cpu_temp_sysfs_or_sensors();
};

}  // namespace auto_battlebot
