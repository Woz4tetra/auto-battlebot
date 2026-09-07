#pragma once

#include <map>
#include <string>
#include <variant>
#include <vector>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"

namespace auto_battlebot {
// Forward declaration for recursive type
struct DiagnosticsValue;

// Type to represent nested data structures
using DiagnosticsData = std::map<std::string, DiagnosticsValue>;

struct DiagnosticsValue
    : std::variant<int, double, std::string, std::vector<int>, std::vector<double>,
                   std::vector<std::string>, DiagnosticsData> {
    using variant::variant;
};

/**
 * @brief Flatten a nested map structure with '/' as separator, keeping values typed.
 *
 * Converts nested structures like:
 *   {"motor": {"temperature": [95, 94, 90], "voltage": 12.5}}
 * Into:
 *   {"motor/temperature/0": 95, "motor/temperature/1": 94, "motor/temperature/2": 90,
 *    "motor/voltage": 12.5}
 */
std::map<std::string, DiagnosticScalar> flatten_diagnostics_data(
    const DiagnosticsData &data, const std::string &parent_key = "",
    const std::string &separator = "/");

}  // namespace auto_battlebot
