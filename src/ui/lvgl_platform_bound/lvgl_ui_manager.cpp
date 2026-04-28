#include "ui/ui_manager.hpp"

#include "diagnostics_logger/ui_diagnostics_backend.hpp"
#include "ui/config.hpp"
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {

UIManager::UIManager(const UiConfiguration& config, double max_loop_rate)
    : ui_state_(std::make_shared<UIState>()) {
    ui_state_->set_window_size(config.width, config.height);
    ui_state_->set_fullscreen(config.fullscreen);
    ui_state_->set_battery_source(config.battery_source);
    ui_state_->set_battery_options(config.battery);
    ui_state_->set_rate_avg_window(config.rate_avg_window);
    ui_state_->set_max_loop_rate(max_loop_rate);
    ui_state_->set_rate_fail_threshold(config.rate_fail_threshold);
    ui_state_->set_rate_fail_duration_sec(config.rate_fail_duration_sec);
    diagnostics_backend_ = std::make_shared<UIDiagnosticsBackend>(ui_state_);
}

std::shared_ptr<UIState> UIManager::ui_state() const { return ui_state_; }

std::shared_ptr<DiagnosticsBackend> UIManager::diagnostics_backend() const {
    return diagnostics_backend_;
}

void UIManager::start() { thread_ = std::jthread(run_ui_thread, ui_state_); }

void UIManager::request_quit() { thread_.request_stop(); }

}  // namespace auto_battlebot
