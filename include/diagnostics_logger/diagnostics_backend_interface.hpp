#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace auto_battlebot {
/** Diagnostic level values (0 OK, 1 WARN, 2 ERROR, 3 STALE, as recorded on /diagnostics/<module>).
 */
namespace DiagnosticLevel {
constexpr int8_t OK = 0;
constexpr int8_t WARN = 1;
constexpr int8_t ERROR = 2;
constexpr int8_t STALE = 3;
}  // namespace DiagnosticLevel

using DiagnosticScalar = std::variant<int, double, std::string>;

std::string diagnostic_scalar_to_string(const DiagnosticScalar &value);

struct DiagnosticStatusSnapshot {
    std::string name;
    std::string subsection;
    int8_t level = DiagnosticLevel::OK;
    std::map<std::string, DiagnosticScalar> values;
    std::string message;
    /** Milliseconds since the subsection was last logged. */
    double age_ms = 0.0;
    /** Not logged in this publish window: the values repeat an earlier message. */
    bool held = false;
};

class DiagnosticsBackend {
   public:
    virtual ~DiagnosticsBackend() = default;
    virtual void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) = 0;
};
}  // namespace auto_battlebot
