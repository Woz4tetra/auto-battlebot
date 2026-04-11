#include "lvgl_platform_bound/lvgl_ui_controller.hpp"

#include <algorithm>

namespace auto_battlebot::ui_internal {

UiController::UiController(std::shared_ptr<UIState> ui_state) : ui_state_(std::move(ui_state)) {}

void UiController::dispatch(const UiCommand &command) const {
    if (!ui_state_) return;
    switch (command.type) {
        case UiCommandType::REQUEST_REINITIALIZE:
            ui_state_->reinit_requested.store(true);
            break;
        case UiCommandType::SET_OPPONENT_COUNT:
            ui_state_->opponent_count_requested.store(std::clamp(command.opponent_count, 1, 3));
            break;
        case UiCommandType::SET_AUTONOMY_ENABLED:
            ui_state_->autonomy_toggle_requested.store(command.autonomy_enabled ? 1 : -1);
            break;
        case UiCommandType::TOGGLE_RECORDING:
            ui_state_->recording_toggle_requested.store(true);
            break;
        case UiCommandType::REQUEST_SYSTEM_ACTION:
            ui_state_->system_action_requested.store(static_cast<int>(command.system_action));
            break;
        case UiCommandType::SET_MANUAL_TARGET:
            ui_state_->set_manual_target(command.manual_target);
            break;
        case UiCommandType::REQUEST_QUIT:
            ui_state_->quit_requested.store(true);
            break;
    }
}

void UiController::request_reinitialize() const {
    UiCommand command{};
    command.type = UiCommandType::REQUEST_REINITIALIZE;
    dispatch(command);
}

void UiController::set_opponent_count(int opponent_count) const {
    UiCommand command{};
    command.type = UiCommandType::SET_OPPONENT_COUNT;
    command.opponent_count = opponent_count;
    dispatch(command);
}

void UiController::toggle_autonomy() const {
    if (!ui_state_) return;
    SystemStatus st;
    ui_state_->get_system_status(st);
    UiCommand command{};
    command.type = UiCommandType::SET_AUTONOMY_ENABLED;
    command.autonomy_enabled = !st.autonomy_enabled;
    dispatch(command);
}

void UiController::toggle_recording() const {
    UiCommand command{};
    command.type = UiCommandType::TOGGLE_RECORDING;
    dispatch(command);
}

void UiController::request_system_action(UISystemAction action) const {
    UiCommand command{};
    command.type = UiCommandType::REQUEST_SYSTEM_ACTION;
    command.system_action = action;
    dispatch(command);
}

void UiController::set_manual_target(const std::optional<TargetSelection> &target) const {
    UiCommand command{};
    command.type = UiCommandType::SET_MANUAL_TARGET;
    command.manual_target = target;
    dispatch(command);
}

void UiController::request_quit() const {
    UiCommand command{};
    command.type = UiCommandType::REQUEST_QUIT;
    dispatch(command);
}

}  // namespace auto_battlebot::ui_internal
