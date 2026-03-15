#pragma once

#include <miniros/publisher.h>

#include <memory>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"

namespace auto_battlebot {
/** Diagnostics backend that publishes to a ROS DiagnosticArray topic. */
class RosDiagnosticsBackend : public DiagnosticsBackend {
   public:
    RosDiagnosticsBackend(std::shared_ptr<miniros::Publisher> publisher,
                          std::shared_ptr<McapRecorder> mcap_recorder = nullptr);
    void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) override;

   private:
    std::shared_ptr<miniros::Publisher> publisher_;
    std::shared_ptr<McapRecorder> mcap_recorder_;
};
}  // namespace auto_battlebot
