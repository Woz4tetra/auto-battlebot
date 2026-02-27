#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"

namespace auto_battlebot {
class UIState;

/** Diagnostics backend that pushes snapshots into UIState for the UI thread. */
class UIDiagnosticsBackend : public DiagnosticsBackend {
   public:
    explicit UIDiagnosticsBackend(std::shared_ptr<UIState> ui_state);
    void receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) override;

   private:
    std::shared_ptr<UIState> ui_state_;
};
}  // namespace auto_battlebot
