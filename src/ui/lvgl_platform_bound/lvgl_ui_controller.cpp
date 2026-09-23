#include "lvgl_platform_bound/lvgl_ui_controller.hpp"

#include <algorithm>

namespace auto_battlebot::ui_internal {

UiController::UiController(std::shared_ptr<UIState> ui_state) : ui_state_(std::move(ui_state)) {}

void UiController::request_reinitialize() const {
    if (ui_state_) ui_state_->post_command(remote::ReinitFieldCommand{});
}

void UiController::set_opponent_count(int opponent_count) const {
    if (!ui_state_) return;
    ui_state_->post_command(
        remote::SetOpponentCountCommand{.count = std::clamp(opponent_count, 1, 3)});
}

void UiController::toggle_autonomy() const {
    if (!ui_state_) return;
    SystemStatus st;
    ui_state_->get_system_status(st);
    ui_state_->post_command(remote::SetAutonomyCommand{.enabled = !st.autonomy_enabled});
}

void UiController::toggle_recording() const {
    if (!ui_state_) return;
    SystemStatus st;
    ui_state_->get_system_status(st);
    // Off only when both are on, so a half-on state turns both on.
    ui_state_->post_command(remote::SetRecordingCommand{.enabled = !st.recording_enabled});
}

void UiController::request_system_action(SystemAction action) const {
    if (ui_state_) ui_state_->post_command(remote::SystemActionCommand{.action = action});
}

void UiController::set_manual_target(const std::optional<TargetSelection> &target) const {
    if (ui_state_) ui_state_->set_manual_target(target);
}

void UiController::select_profile(const std::string &profile_name) const {
    if (ui_state_) ui_state_->post_command(remote::SelectProfileCommand{.name = profile_name});
}

void UiController::request_quit() const {
    if (ui_state_) ui_state_->quit_requested.store(true);
}

}  // namespace auto_battlebot::ui_internal
