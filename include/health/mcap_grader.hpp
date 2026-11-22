#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "health/performance_monitor.hpp"

namespace auto_battlebot {

struct McapGrade {
    std::filesystem::path path;
    double duration_s = 0.0;
    std::optional<std::string> transmitter_type;
    std::vector<PerformanceWarning> warnings;
    std::vector<std::string> notes;
    std::map<std::string, uint64_t> topic_counts;
    std::map<std::string, uint64_t> link_totals;

    bool red() const;
};

McapGrade grade_mcap(const std::filesystem::path &path,
                     const PerformanceMonitorConfiguration &config);

std::string format_grade(const McapGrade &grade);

}  // namespace auto_battlebot
