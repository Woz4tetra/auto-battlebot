#pragma once

#include <memory>

namespace auto_battlebot
{
    class UIState;

    /** Run the ImGui UI on the current thread (call from a dedicated UI thread). Fullscreen, touch-friendly. */
    void run_ui_thread(std::shared_ptr<UIState> ui_state);
} // namespace auto_battlebot
