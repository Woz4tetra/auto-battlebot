#include "diagnostics_logger/ros_diagnostics_backend.hpp"

#include <miniros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.hxx>
#include <diagnostic_msgs/KeyValue.hxx>

namespace auto_battlebot {
RosDiagnosticsBackend::RosDiagnosticsBackend(std::shared_ptr<miniros::Publisher> publisher,
                                             std::shared_ptr<McapRecorder> mcap_recorder)
    : publisher_(std::move(publisher)), mcap_recorder_(std::move(mcap_recorder)) {}

void RosDiagnosticsBackend::receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) {
    if (!publisher_ || snapshots.empty()) {
        return;
    }
    diagnostic_msgs::DiagnosticArray msg;
    msg.header.stamp = miniros::Time::now();
    msg.header.frame_id = "";
    for (const auto &snap : snapshots) {
        diagnostic_msgs::DiagnosticStatus status;
        status.level = snap.level;
        status.name = snap.subsection.empty() ? snap.name : snap.subsection;
        status.hardware_id = snap.name;
        status.message = snap.message;
        for (const auto &[key, value] : snap.values) {
            diagnostic_msgs::KeyValue kv;
            kv.key = key;
            kv.value = value;
            status.values.push_back(kv);
        }
        msg.status.push_back(status);
    }
    publisher_->publish(msg);
    if (mcap_recorder_) mcap_recorder_->write("/diagnostics", msg);
}
}  // namespace auto_battlebot
