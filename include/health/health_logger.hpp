#pragma once

#include <chrono>
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

   private:
    HealthConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::chrono::steady_clock::time_point last_sample_time_;
    bool has_sampled_ = false;

    static bool is_x86();
    static bool command_exists(const std::string& command);
    static std::string trim(const std::string& value);
    static std::vector<std::string> split(const std::string& input, char delimiter);
    static bool run_command(const std::string& command, std::string& output);
    static bool parse_double(const std::string& value, double& out);
    static bool parse_int(const std::string& value, int& out);

    bool collect_tegrastats();
    bool collect_x86_health();
    bool collect_nvidia_smi();
    bool collect_cpu_temp_sysfs_or_sensors();
};

}  // namespace auto_battlebot
