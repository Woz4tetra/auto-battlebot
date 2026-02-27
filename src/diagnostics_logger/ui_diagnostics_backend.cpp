#include "diagnostics_logger/ui_diagnostics_backend.hpp"

#include "ui/ui_state.hpp"

namespace auto_battlebot {
UIDiagnosticsBackend::UIDiagnosticsBackend(std::shared_ptr<UIState> ui_state)
    : ui_state_(std::move(ui_state)) {}

void UIDiagnosticsBackend::receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) {
    if (!ui_state_) {
        return;
    }
    ui_state_->set_diagnostic_snapshots(snapshots);

    std::map<std::string, std::string> merged;
    for (const auto &snap : snapshots) {
        std::string prefix =
            snap.name.empty()
                ? ""
                : (snap.name + (snap.subsection.empty() ? "." : "." + snap.subsection + "."));
        for (const auto &[key, value] : snap.values) {
            merged[prefix + key] = value;
        }
        if (!snap.message.empty()) {
            merged[prefix + "message"] = snap.message;
        }
    }
    ui_state_->set_diagnostics(merged);
}
}  // namespace auto_battlebot
