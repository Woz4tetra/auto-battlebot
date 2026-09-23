#pragma once

#include <memory>
#include <optional>
#include <string>

#include "data_structures/target_selection.hpp"
#include "enums/system_action.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot::ui_internal {

/** Turns LVGL tile presses into remote commands on the shared queue, the same commands the web
 *  UI sends. Toggles read the current SystemStatus and post the explicit new value. */
class UiController {
   public:
    explicit UiController(std::shared_ptr<UIState> ui_state);

    void request_reinitialize() const;
    void set_opponent_count(int opponent_count) const;
    void toggle_autonomy() const;
    void toggle_recording() const;
    void request_system_action(SystemAction action) const;
    void set_manual_target(const std::optional<TargetSelection> &target) const;
    void select_profile(const std::string &profile_name) const;
    void request_quit() const;

   private:
    std::shared_ptr<UIState> ui_state_;
};

}  // namespace auto_battlebot::ui_internal
