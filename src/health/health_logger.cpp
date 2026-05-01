#include "health/health_logger.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "spdlog/spdlog.h"

namespace auto_battlebot {
namespace {
constexpr double kTegrastatsCollectWarnMs = 200.0;

bool is_regular_executable(const std::filesystem::path& path) {
    return std::filesystem::exists(path) && std::filesystem::is_regular_file(path) &&
           access(path.c_str(), X_OK) == 0;
}

std::string normalize_metric_key(const std::string& raw) {
    std::string out;
    out.reserve(raw.size());
    bool last_was_underscore = false;
    for (char ch : raw) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch)) {
            out.push_back(static_cast<char>(std::tolower(uch)));
            last_was_underscore = false;
            continue;
        }
        if (!last_was_underscore) {
            out.push_back('_');
            last_was_underscore = true;
        }
    }
    while (!out.empty() && out.front() == '_') {
        out.erase(out.begin());
    }
    while (!out.empty() && out.back() == '_') {
        out.pop_back();
    }
    return out;
}

}  // namespace

HealthLogger::HealthLogger(const HealthConfiguration& config) : config_(config) {
    if (DiagnosticsLogger::is_initialized()) {
        logger_ = DiagnosticsLogger::get_logger("health");
    }
}

void HealthLogger::maybe_log() {
    perform_sampling();
    emit_heartbeat_if_due(std::chrono::steady_clock::now());
}

void HealthLogger::record_tick(double tick_ms) {
    heartbeat_ticks_++;
    max_tick_ms_ = std::max(max_tick_ms_, tick_ms);
}

void HealthLogger::perform_sampling() {
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
        if (!compute_mode_initialized_) {
            collect_compute_mode();
            compute_mode_initialized_ = true;
        }
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

void HealthLogger::emit_heartbeat_if_due(std::chrono::steady_clock::time_point now) {
    if (!heartbeat_window_started_) {
        heartbeat_window_start_ = now;
        heartbeat_window_started_ = true;
        return;
    }
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - heartbeat_window_start_)
            .count();
    if (elapsed_ms < 1000) {
        return;
    }
    spdlog::debug("runner heartbeat: ticks={} tick_ms_max={:.2f}", heartbeat_ticks_, max_tick_ms_);
    heartbeat_window_start_ = now;
    heartbeat_ticks_ = 0;
    max_tick_ms_ = 0.0;
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

bool HealthLogger::start_tegrastats_stream() {
    if (tegrastats_pipe_) {
        return true;
    }
    if (tegrastats_unavailable_) {
        return false;
    }
    if (!command_exists("tegrastats")) {
        tegrastats_unavailable_ = true;
        DiagnosticsData data;
        data["enabled"] = config_.tegrastats_enable ? 1 : 0;
        data["available"] = 0;
        logger_->warning("tegrastats", data, "tegrastats unavailable");
        return false;
    }

    const int interval_ms = std::max(250, config_.sample_period_ms);
    const std::string command = "tegrastats --interval " + std::to_string(interval_ms) + " 2>&1";
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        DiagnosticsData data;
        data["enabled"] = config_.tegrastats_enable ? 1 : 0;
        data["available"] = 0;
        logger_->warning("tegrastats", data, "failed to start tegrastats stream");
        return false;
    }
    tegrastats_pipe_ = std::shared_ptr<FILE>(pipe, [](FILE* p) {
        if (p) {
            pclose(p);
        }
    });

    return true;
}

void HealthLogger::stop_tegrastats_stream() { tegrastats_pipe_.reset(); }

bool HealthLogger::collect_tegrastats() {
    const auto collect_start = std::chrono::steady_clock::now();
    DiagnosticsData data;
    data["enabled"] = config_.tegrastats_enable ? 1 : 0;

    if (!start_tegrastats_stream()) {
        return false;
    }

    data["available"] = 1;
    std::string line;
    pollfd pfd{};
    pfd.fd = fileno(tegrastats_pipe_.get());
    pfd.events = POLLIN;
    const int ready = poll(&pfd, 1, 250);
    if (ready <= 0 || (pfd.revents & POLLIN) == 0) {
        logger_->warning("tegrastats", data, "tegrastats produced no line");
        return false;
    }

    char buffer[2048];
    int lines_read = 0;
    while (fgets(buffer, sizeof(buffer), tegrastats_pipe_.get()) != nullptr) {
        lines_read++;
        const std::string candidate = trim(buffer);
        if (!candidate.empty()) {
            line = candidate;
        }

        pfd.revents = 0;
        const int has_more = poll(&pfd, 1, 0);
        if (has_more <= 0 || (pfd.revents & POLLIN) == 0) {
            break;
        }
    }

    const double collect_elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - collect_start)
            .count();
    data["collect_elapsed_ms"] = collect_elapsed_ms;
    data["lines_read"] = lines_read;

    if (line.empty()) {
        if (feof(tegrastats_pipe_.get())) {
            stop_tegrastats_stream();
            logger_->warning("tegrastats", data, "tegrastats stream ended");
            return false;
        }
        logger_->warning("tegrastats", data, "tegrastats returned empty output");
        return false;
    }

    bool parse_ok = false;

    std::smatch match;
    const std::regex ram_regex(R"(RAM\s+(\d+)\/(\d+)MB(?:\s+\(lfb\s+(\d+)x(\d+)MB\))?)");
    if (std::regex_search(line, match, ram_regex) && match.size() >= 3) {
        int used_mb = 0;
        int total_mb = 0;
        if (parse_int(match[1].str(), used_mb)) {
            data["ram_used_mb"] = used_mb;
            parse_ok = true;
        }
        if (parse_int(match[2].str(), total_mb)) {
            data["ram_total_mb"] = total_mb;
            parse_ok = true;
        }
        if (match.size() >= 5) {
            int lfb_blocks = 0;
            int lfb_block_size_mb = 0;
            if (parse_int(match[3].str(), lfb_blocks)) {
                data["ram_lfb_blocks"] = lfb_blocks;
                parse_ok = true;
            }
            if (parse_int(match[4].str(), lfb_block_size_mb)) {
                data["ram_lfb_block_size_mb"] = lfb_block_size_mb;
                parse_ok = true;
            }
        }
    }

    const std::regex swap_regex(R"(SWAP\s+(\d+)\/(\d+)MB(?:\s+\(cached\s+(\d+)MB\))?)");
    if (std::regex_search(line, match, swap_regex) && match.size() >= 3) {
        int used_mb = 0;
        int total_mb = 0;
        if (parse_int(match[1].str(), used_mb)) {
            data["swap_used_mb"] = used_mb;
            parse_ok = true;
        }
        if (parse_int(match[2].str(), total_mb)) {
            data["swap_total_mb"] = total_mb;
            parse_ok = true;
        }
        if (match.size() >= 4) {
            int cached_mb = 0;
            if (parse_int(match[3].str(), cached_mb)) {
                data["swap_cached_mb"] = cached_mb;
                parse_ok = true;
            }
        }
    }

    const std::regex cpu_regex(R"(CPU\s+\[([^\]]+)\])");
    if (std::regex_search(line, match, cpu_regex) && match.size() >= 2) {
        const std::vector<std::string> cpu_entries = split(match[1].str(), ',');
        const std::regex cpu_core_regex(R"((\d+)%@(\d+))");
        int total_cores = static_cast<int>(cpu_entries.size());
        int active_cores = 0;
        double util_sum = 0.0;
        int util_count = 0;
        double freq_sum_mhz = 0.0;
        int freq_count = 0;
        for (size_t i = 0; i < cpu_entries.size(); ++i) {
            const std::string token = trim(cpu_entries[i]);
            std::smatch cpu_match;
            if (!std::regex_match(token, cpu_match, cpu_core_regex) || cpu_match.size() < 3) {
                continue;
            }
            int core_util = 0;
            int core_freq_mhz = 0;
            if (parse_int(cpu_match[1].str(), core_util)) {
                data["cpu" + std::to_string(i) + "_util_percent"] = core_util;
                util_sum += core_util;
                util_count++;
                parse_ok = true;
            }
            if (parse_int(cpu_match[2].str(), core_freq_mhz)) {
                data["cpu" + std::to_string(i) + "_freq_mhz"] = core_freq_mhz;
                freq_sum_mhz += core_freq_mhz;
                freq_count++;
                parse_ok = true;
            }
            active_cores++;
        }
        data["cpu_core_count"] = total_cores;
        data["cpu_active_core_count"] = active_cores;
        if (util_count > 0) {
            data["cpu_avg_util_percent"] = util_sum / static_cast<double>(util_count);
        }
        if (freq_count > 0) {
            data["cpu_avg_freq_mhz"] = freq_sum_mhz / static_cast<double>(freq_count);
        }
    }

    const std::regex gr3d_regex(R"(GR3D_FREQ\s+(\d+)%(?:@(\d+))?)");
    if (std::regex_search(line, match, gr3d_regex) && match.size() >= 2) {
        int gr3d = 0;
        if (parse_int(match[1].str(), gr3d)) {
            data["gpu_util_percent"] = gr3d;
            parse_ok = true;
        }
        if (match.size() >= 3) {
            int gr3d_freq_mhz = 0;
            if (parse_int(match[2].str(), gr3d_freq_mhz)) {
                data["gpu_freq_mhz"] = gr3d_freq_mhz;
                parse_ok = true;
            }
        }
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
        parse_ok = true;
        const std::string label = (*it)[1].str();
        const std::string label_key = normalize_metric_key(label);
        if (!label_key.empty()) {
            data["temp_" + label_key + "_c"] = temp;
        }
        if (label_key == "gpu") {
            data["gpu_temp_c"] = temp;
        }
    }
    if (temp_count > 0) {
        data["max_temp_c"] = max_temp_c;
        data["temp_sensor_count"] = temp_count;
        last_temp_c_ = max_temp_c;
    }

    const std::regex rail_regex(
        R"(([A-Z][A-Z0-9_]+)\s+([0-9]+(?:\.[0-9]+)?)mW\/([0-9]+(?:\.[0-9]+)?)mW)");
    std::sregex_iterator rail_begin(line.begin(), line.end(), rail_regex);
    int rail_count = 0;
    double rail_now_total_mw = 0.0;
    double rail_avg_total_mw = 0.0;
    for (auto it = rail_begin; it != end; ++it) {
        if (it->size() < 4) continue;
        const std::string rail_name = (*it)[1].str();
        const std::string rail_key = normalize_metric_key(rail_name);
        if (rail_key.empty()) continue;
        double now_mw = 0.0;
        double avg_mw = 0.0;
        bool parsed_any = false;
        if (parse_double((*it)[2].str(), now_mw)) {
            data["power_" + rail_key + "_mw"] = now_mw;
            rail_now_total_mw += now_mw;
            parsed_any = true;
        }
        if (parse_double((*it)[3].str(), avg_mw)) {
            data["power_" + rail_key + "_avg_mw"] = avg_mw;
            rail_avg_total_mw += avg_mw;
            parsed_any = true;
        }
        if (parsed_any) {
            rail_count++;
            parse_ok = true;
        }
    }
    if (rail_count > 0) {
        data["power_rail_count"] = rail_count;
        data["power_total_mw"] = rail_now_total_mw;
        data["power_total_avg_mw"] = rail_avg_total_mw;
    }

    data["parse_ok"] = parse_ok ? 1 : 0;
    if (!parse_ok) {
        data["raw_line"] = line;
        logger_->warning("tegrastats", data, "tegrastats line parsed no known metrics");
        return false;
    }

    if (collect_elapsed_ms > kTegrastatsCollectWarnMs) {
        spdlog::warn("validation: collect_tegrastats slow elapsed_ms={:.2f} lines_read={}",
                     collect_elapsed_ms, lines_read);
    }
    logger_->debug("tegrastats", data);
    return true;
}

void HealthLogger::collect_compute_mode() {
    if (!command_exists("nvpmodel")) return;
    std::string output;
    if (!run_command("nvpmodel -q 2>/dev/null", output)) return;
    // Output is typically:
    //   NV Power Mode: MAXN
    //   0: MAXN
    // Extract the mode name from the first "NV Power Mode: <NAME>" line.
    const std::regex mode_regex(R"(NV Power Mode:\s*(\S+))");
    std::smatch match;
    if (std::regex_search(output, match, mode_regex) && match.size() >= 2) {
        last_compute_mode_ = match[1].str();
    }
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
            "nvidia-smi "
            "--query-gpu=temperature.gpu,utilization.gpu,memory.used,memory.total,power.draw "
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
