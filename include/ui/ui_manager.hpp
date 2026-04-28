#pragma once

#include <memory>
#include <thread>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "quittable.hpp"

namespace auto_battlebot {

class UIState;
struct UiConfiguration;

/** Owns the UIState, its diagnostics backend, and the UI thread lifetime.
 *  Construct early (before DiagnosticsLogger::initialize) so the backend can be
 *  registered, then call start() when ready to show the window. */
class UIManager : public Quittable {
   public:
    UIManager(const UiConfiguration& config, double max_loop_rate);

    UIManager(const UIManager&) = delete;
    UIManager& operator=(const UIManager&) = delete;

    /** UIState shared with Runner for status updates and debug image. */
    std::shared_ptr<UIState> ui_state() const;

    /** DiagnosticsBackend to register before DiagnosticsLogger::initialize(). */
    std::shared_ptr<DiagnosticsBackend> diagnostics_backend() const;

    /** Spawns the UI thread. Must be called after all UIState config setters. */
    void start();

    /** Quittable: requests the UI thread to stop. */
    void request_quit() override;

   private:
    std::shared_ptr<UIState> ui_state_;
    std::shared_ptr<DiagnosticsBackend> diagnostics_backend_;
    std::jthread thread_;
};

}  // namespace auto_battlebot
