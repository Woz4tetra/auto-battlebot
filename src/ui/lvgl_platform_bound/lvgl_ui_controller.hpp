#pragma once

#include <memory>
#include <optional>

#include "data_structures/target_selection.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot::ui_internal {

enum class UiCommandType {
    REQUEST_REINITIALIZE,
    SET_OPPONENT_COUNT,
    SET_AUTONOMY_ENABLED,
    TOGGLE_RECORDING,
    REQUEST_SYSTEM_ACTION,
    SET_MANUAL_TARGET,
    REQUEST_QUIT,
};

struct UiCommand {
    UiCommandType type;
    int opponent_count = -1;
    bool autonomy_enabled = false;
    UISystemAction system_action = UISystemAction::NONE;
    std::optional<TargetSelection> manual_target;
};

class UiController {
   public:
    explicit UiController(std::shared_ptr<UIState> ui_state);

    void dispatch(const UiCommand &command) const;
    void request_reinitialize() const;
    void set_opponent_count(int opponent_count) const;
    void toggle_autonomy() const;
    void toggle_recording() const;
    void request_system_action(UISystemAction action) const;
    void set_manual_target(const std::optional<TargetSelection> &target) const;
    void request_quit() const;

   private:
    std::shared_ptr<UIState> ui_state_;
};

}  // namespace auto_battlebot::ui_internal
