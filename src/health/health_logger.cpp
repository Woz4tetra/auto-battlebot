#include "health/health_logger.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {
namespace {

bool is_regular_executable(const std::filesystem::path& path) {
    return std::filesystem::exists(path) && std::filesystem::is_regular_file(path) &&
           access(path.c_str(), X_OK) == 0;
}

}  // namespace

HealthLogger::HealthLogger(const HealthConfiguration& config) : config_(config) {
    if (DiagnosticsLogger::is_initialized()) {
        logger_ = DiagnosticsLogger::get_logger("health");
    }
}

void HealthLogger::maybe_log() {
    if (!config_.enable) {
        return;
    }
    if (!logger_) {
        if (!DiagnosticsLogger::is_initialized()) {
            return;
        }
        logger_ = DiagnosticsLogger::get_logger("health");
        if (!logger_) {
            return;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    if (has_sampled_) {
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sample_time_).count();
        if (elapsed_ms < config_.sample_period_ms) {
            return;
        }
    }
    has_sampled_ = true;
    last_sample_time_ = now;

    bool any_logged = false;
    if (config_.tegrastats_enable) {
        any_logged = collect_tegrastats() || any_logged;
    }

    if (config_.x86_tools_enable && is_x86()) {
        any_logged = collect_x86_health() || any_logged;
    }

    DiagnosticsData status;
    status["enable"] = config_.enable ? 1 : 0;
    status["tegrastats_enable"] = config_.tegrastats_enable ? 1 : 0;
    status["x86_tools_enable"] = config_.x86_tools_enable ? 1 : 0;
    status["sample_period_ms"] = config_.sample_period_ms;
    status["any_source_logged"] = any_logged ? 1 : 0;
    logger_->debug("status", status);
}

bool HealthLogger::is_x86() {
#if defined(__x86_64__) || defined(__i386__)
    return true;
#else
    return false;
#endif
}

bool HealthLogger::command_exists(const std::string& command) {
    if (command.empty()) return false;
    if (command.find('/') != std::string::npos) {
        return is_regular_executable(command);
    }

    const char* path_env = std::getenv("PATH");
    if (!path_env) return false;

    std::stringstream path_stream(path_env);
    std::string dir;
    while (std::getline(path_stream, dir, ':')) {
        if (dir.empty()) continue;
        std::filesystem::path candidate = std::filesystem::path(dir) / command;
        if (is_regular_executable(candidate)) {
            return true;
        }
    }
    return false;
}

std::string HealthLogger::trim(const std::string& value) {
    const size_t start = value.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) {
        return "";
    }
    const size_t end = value.find_last_not_of(" \t\n\r");
    return value.substr(start, end - start + 1);
}

std::vector<std::string> HealthLogger::split(const std::string& input, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        parts.push_back(item);
    }
    return parts;
}

bool HealthLogger::run_command(const std::string& command, std::string& output) {
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return false;

    output.clear();
    char buffer[512];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }

    const int rc = pclose(pipe);
    return rc == 0;
}

bool HealthLogger::parse_double(const std::string& value, double& out) {
    try {
        size_t idx = 0;
        out = std::stod(value, &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

bool HealthLogger::parse_int(const std::string& value, int& out) {
    try {
        size_t idx = 0;
        out = std::stoi(value, &idx);
        return idx > 0;
    } catch (...) {
        return false;
    }
}

bool HealthLogger::collect_tegrastats() {
    DiagnosticsData data;
    data["enabled"] = config_.tegrastats_enable ? 1 : 0;

    if (!command_exists("tegrastats")) {
        data["available"] = 0;
        logger_->warning("tegrastats", data, "tegrastats unavailable");
        return false;
    }

    std::string output;
    if (!run_command("tegrastats --interval 1000 --count 1 2>/dev/null", output)) {
        data["available"] = 1;
        logger_->warning("tegrastats", data, "tegrastats command failed");
        return false;
    }

    const std::string line = trim(output);
    data["available"] = 1;
    if (line.empty()) {
        logger_->warning("tegrastats", data, "tegrastats returned empty output");
        return false;
    }

    std::smatch match;
    const std::regex ram_regex(R"(RAM\s+(\d+)\/(\d+)MB)");
    if (std::regex_search(line, match, ram_regex) && match.size() >= 3) {
        int used_mb = 0;
        int total_mb = 0;
        if (parse_int(match[1].str(), used_mb)) data["ram_used_mb"] = used_mb;
        if (parse_int(match[2].str(), total_mb)) data["ram_total_mb"] = total_mb;
    }

    const std::regex gr3d_regex(R"(GR3D_FREQ\s+(\d+)%)");
    if (std::regex_search(line, match, gr3d_regex) && match.size() >= 2) {
        int gr3d = 0;
        if (parse_int(match[1].str(), gr3d)) data["gpu_util_percent"] = gr3d;
    }

    const std::regex temp_regex(R"(([A-Za-z0-9_]+)@([0-9]+(?:\.[0-9]+)?)C)");
    std::sregex_iterator begin(line.begin(), line.end(), temp_regex);
    std::sregex_iterator end;
    double max_temp_c = -1.0;
    int temp_count = 0;
    for (auto it = begin; it != end; ++it) {
        if (it->size() < 3) continue;
        double temp = 0.0;
        if (!parse_double((*it)[2].str(), temp)) continue;
        max_temp_c = std::max(max_temp_c, temp);
        temp_count++;
        const std::string label = (*it)[1].str();
        if (label == "GPU") {
            data["gpu_temp_c"] = temp;
        }
    }
    if (temp_count > 0) {
        data["max_temp_c"] = max_temp_c;
        data["temp_sensor_count"] = temp_count;
    }

    logger_->debug("tegrastats", data);
    return true;
}

bool HealthLogger::collect_x86_health() {
    bool any = false;
    any = collect_nvidia_smi() || any;
    any = collect_cpu_temp_sysfs_or_sensors() || any;
    if (!any) {
        DiagnosticsData data;
        data["x86_tools_enable"] = config_.x86_tools_enable ? 1 : 0;
        logger_->warning("x86", data, "x86 health tools unavailable");
    }
    return any;
}

bool HealthLogger::collect_nvidia_smi() {
    DiagnosticsData data;
    data["attempted"] = 1;
    if (!command_exists("nvidia-smi")) {
        data["available"] = 0;
        logger_->debug("x86_nvidia_smi", data, "nvidia-smi not available; skipping");
        return false;
    }

    std::string output;
    if (!run_command(
            "nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,memory.used,memory.total,power.draw "
            "--format=csv,noheader,nounits 2>/dev/null",
            output)) {
        data["available"] = 1;
        logger_->warning("x86_nvidia_smi", data, "nvidia-smi execution failed");
        return false;
    }

    std::vector<std::string> lines = split(trim(output), '\n');
    if (lines.empty() || trim(lines[0]).empty()) {
        data["available"] = 1;
        logger_->warning("x86_nvidia_smi", data, "nvidia-smi returned empty output");
        return false;
    }

    std::vector<std::string> fields = split(lines[0], ',');
    if (fields.size() >= 5) {
        double temp_c = 0.0;
        double gpu_util = 0.0;
        double mem_used_mb = 0.0;
        double mem_total_mb = 0.0;
        double power_w = 0.0;
        if (parse_double(trim(fields[0]), temp_c)) data["gpu_temp_c"] = temp_c;
        if (parse_double(trim(fields[1]), gpu_util)) data["gpu_util_percent"] = gpu_util;
        if (parse_double(trim(fields[2]), mem_used_mb)) data["gpu_mem_used_mb"] = mem_used_mb;
        if (parse_double(trim(fields[3]), mem_total_mb)) data["gpu_mem_total_mb"] = mem_total_mb;
        if (parse_double(trim(fields[4]), power_w)) data["gpu_power_w"] = power_w;
    }
    data["available"] = 1;
    logger_->debug("x86_nvidia_smi", data);
    return true;
}

bool HealthLogger::collect_cpu_temp_sysfs_or_sensors() {
    DiagnosticsData data;
    std::vector<double> temps;

    const std::filesystem::path thermal_dir("/sys/class/thermal");
    if (std::filesystem::exists(thermal_dir) && std::filesystem::is_directory(thermal_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(thermal_dir)) {
            const auto name = entry.path().filename().string();
            if (name.rfind("thermal_zone", 0) != 0) continue;

            const auto temp_file = entry.path() / "temp";
            std::ifstream in(temp_file);
            if (!in.is_open()) continue;
            std::string raw;
            std::getline(in, raw);
            raw = trim(raw);
            if (raw.empty()) continue;
            double value = 0.0;
            if (!parse_double(raw, value)) continue;
            if (value > 200.0) value /= 1000.0;
            if (value < -40.0 || value > 200.0) continue;
            temps.push_back(value);
        }
    }

    if (temps.empty() && command_exists("sensors")) {
        std::string output;
        if (run_command("sensors -u 2>/dev/null", output)) {
            const std::regex input_regex(R"(_input:\s*([0-9]+(?:\.[0-9]+)?))");
            std::sregex_iterator begin(output.begin(), output.end(), input_regex);
            std::sregex_iterator end;
            for (auto it = begin; it != end; ++it) {
                if (it->size() < 2) continue;
                double value = 0.0;
                if (!parse_double((*it)[1].str(), value)) continue;
                if (value < -40.0 || value > 200.0) continue;
                temps.push_back(value);
            }
            data["source"] = "sensors";
        }
    } else if (!temps.empty()) {
        data["source"] = "sysfs_thermal";
    }

    if (temps.empty()) {
        data["available"] = 0;
        logger_->debug("x86_cpu_temp", data, "No CPU thermal data available");
        return false;
    }

    double max_temp = temps[0];
    double min_temp = temps[0];
    double sum = 0.0;
    for (double t : temps) {
        max_temp = std::max(max_temp, t);
        min_temp = std::min(min_temp, t);
        sum += t;
    }
    data["available"] = 1;
    data["sensor_count"] = static_cast<int>(temps.size());
    data["max_temp_c"] = max_temp;
    data["min_temp_c"] = min_temp;
    data["avg_temp_c"] = sum / static_cast<double>(temps.size());
    logger_->debug("x86_cpu_temp", data);
    return true;
}

}  // namespace auto_battlebot
