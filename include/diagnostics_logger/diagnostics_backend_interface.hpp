#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace auto_battlebot {
enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

using DiagnosticScalar = std::variant<int, double, std::string>;

std::string diagnostic_scalar_to_string(const DiagnosticScalar &value);

struct DiagnosticStatusSnapshot {
    std::string name;
    std::string subsection;
    DiagnosticLevel level = DiagnosticLevel::OK;
    std::map<std::string, DiagnosticScalar> values;
    std::string message;
    double ms_since_last_logged = 0.0;
    bool is_repeat_of_earlier_message = false;
};

class DiagnosticsBackend {
   public:
    virtual ~DiagnosticsBackend() = default;
    virtual void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) = 0;
};
}  // namespace auto_battlebot
