#pragma once

#include <map>
#include <string>
#include <vector>
#include <variant>
#include <sstream>
#include <diagnostic_msgs/KeyValue.hxx>
#include <diagnostic_msgs/DiagnosticStatus.hxx>

namespace auto_battlebot
{
    // Forward declaration for recursive type
    struct DiagnosticsValue;

    // Type to represent nested data structures
    using DiagnosticsData = std::map<std::string, DiagnosticsValue>;
    ;

    struct DiagnosticsValue : std::variant<
                                  int,
                                  double,
                                  std::string,
                                  std::vector<int>,
                                  std::vector<double>,
                                  std::vector<std::string>,
                                  DiagnosticsData>
    {
        using variant::variant;
    };

    /**
     * @brief Flatten a nested map structure with '/' as separator
     *
     * Converts nested structures like:
     *   {"motor": {"temperature": [95, 94, 90], "voltage": 12.5}}
     * Into:
     *   {"motor/temperature/0": "95", "motor/temperature/1": "94", "motor/temperature/2": "90", "motor/voltage": "12.5"}
     */
    std::map<std::string, std::string> flatten_diagnostics_data(
        const DiagnosticsData &data,
        const std::string &parent_key = "",
        const std::string &separator = "/");

    /**
     * @brief Convert a flat map to DiagnosticStatus message
     *
     * @param data Flattened key-value pairs
     * @param level Diagnostic level (OK, WARN, ERROR, STALE)
     * @param name Name of the diagnostic status
     * @param message Optional message string
     * @param hardware_id Optional hardware ID
     * @return diagnostic_msgs::DiagnosticStatus
     */
    diagnostic_msgs::DiagnosticStatus dict_to_diagnostics(
        const DiagnosticsData &data,
        int8_t level = diagnostic_msgs::DiagnosticStatus::OK,
        const std::string &name = "",
        const std::string &message = "",
        const std::string &hardware_id = "");

} // namespace auto_battlebot
