#pragma once

#include <miniros/publisher.h>

#include <memory>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"

namespace auto_battlebot {
/** Diagnostics backend that publishes to a ROS DiagnosticArray topic. */
class RosDiagnosticsBackend : public DiagnosticsBackend {
   public:
    explicit RosDiagnosticsBackend(std::shared_ptr<miniros::Publisher> publisher);
    void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) override;

   private:
    std::shared_ptr<miniros::Publisher> publisher_;
};
}  // namespace auto_battlebot
