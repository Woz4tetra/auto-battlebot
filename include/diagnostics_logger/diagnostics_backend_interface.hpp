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

/** One flattened diagnostics value. Numbers stay numbers all the way to the recording; only
 *  the UI stringifies, at render time. */
using DiagnosticScalar = std::variant<int, double, std::string>;

std::string diagnostic_scalar_to_string(const DiagnosticScalar &value);

/** Snapshot of one diagnostic status for backend consumption. */
struct DiagnosticStatusSnapshot {
    std::string name;       /** Logger / category name */
    std::string subsection; /** Subsection within the logger (e.g. "tick", "camera.get") */
    int8_t level = DiagnosticLevel::OK;
    std::map<std::string, DiagnosticScalar> values;
    std::string message;
};

/** Interface for diagnostics backends (e.g. UI, Foxglove). */
class DiagnosticsBackend {
   public:
    virtual ~DiagnosticsBackend() = default;
    virtual void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) = 0;
};
}  // namespace auto_battlebot
