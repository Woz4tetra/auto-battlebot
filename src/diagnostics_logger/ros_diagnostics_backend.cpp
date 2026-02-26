#include "diagnostics_logger/ros_diagnostics_backend.hpp"
#include <diagnostic_msgs/DiagnosticArray.hxx>
#include <diagnostic_msgs/KeyValue.hxx>
#include <miniros/ros.h>

namespace auto_battlebot
{
    RosDiagnosticsBackend::RosDiagnosticsBackend(std::shared_ptr<miniros::Publisher> publisher)
        : publisher_(std::move(publisher))
    {
    }

    void RosDiagnosticsBackend::receive(const std::vector<DiagnosticStatusSnapshot> &snapshots)
    {
        if (!publisher_ || snapshots.empty())
        {
            return;
        }
        diagnostic_msgs::DiagnosticArray msg;
        msg.header.stamp = miniros::Time::now();
        msg.header.frame_id = "";
        for (const auto &snap : snapshots)
        {
            diagnostic_msgs::DiagnosticStatus status;
            status.level = snap.level;
            status.name = snap.subsection.empty() ? snap.name : snap.subsection;
            status.hardware_id = snap.name;
            status.message = snap.message;
            for (const auto &[key, value] : snap.values)
            {
                diagnostic_msgs::KeyValue kv;
                kv.key = key;
                kv.value = value;
                status.values.push_back(kv);
            }
            msg.status.push_back(status);
        }
        publisher_->publish(msg);
    }
} // namespace auto_battlebot
