#pragma once

#include <map>
#include <string>
#include <variant>
#include <vector>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"

namespace auto_battlebot {
struct DiagnosticsValue;

using DiagnosticsData = std::map<std::string, DiagnosticsValue>;

struct DiagnosticsValue
    : std::variant<int, double, std::string, std::vector<int>, std::vector<double>,
                   std::vector<std::string>, DiagnosticsData> {
    using variant::variant;
};

std::map<std::string, DiagnosticScalar> flatten_diagnostics_data(
    const DiagnosticsData &data, const std::string &parent_key = "",
    const std::string &separator = "/");

}  // namespace auto_battlebot
