#pragma once

#include <memory>
#include <stop_token>

namespace auto_battlebot {
class UIState;

/** Runs the LVGL UI in the current thread (call from UI thread).
 *  Exits when stop is requested or quit_requested is set on ui_state. */
void run_ui_thread(std::stop_token stop, std::shared_ptr<UIState> ui_state);
}  // namespace auto_battlebot
