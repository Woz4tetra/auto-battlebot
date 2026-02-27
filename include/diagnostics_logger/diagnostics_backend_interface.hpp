#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace auto_battlebot {
/** Diagnostic level values (match diagnostic_msgs::DiagnosticStatus for ROS backend). */
namespace DiagnosticLevel {
constexpr int8_t OK = 0;
constexpr int8_t WARN = 1;
constexpr int8_t ERROR = 2;
constexpr int8_t STALE = 3;
}  // namespace DiagnosticLevel

/** ROS-free snapshot of one diagnostic status for backend consumption. */
struct DiagnosticStatusSnapshot {
    std::string name;       /** Logger / category name */
    std::string subsection; /** Subsection within the logger (e.g. "tick", "camera.get") */
    int8_t level = DiagnosticLevel::OK;
    std::map<std::string, std::string> values;
    std::string message;
};

/** Interface for diagnostics backends (e.g. UI, ROS). */
class DiagnosticsBackend {
   public:
    virtual ~DiagnosticsBackend() = default;
    virtual void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) = 0;
};
}  // namespace auto_battlebot
