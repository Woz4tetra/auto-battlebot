#pragma once

#include <memory>

namespace auto_battlebot {
class UIState;

/** Runs the LVGL UI in the current thread (call from UI thread). */
void run_ui_thread(std::shared_ptr<UIState> ui_state);
}  // namespace auto_battlebot
