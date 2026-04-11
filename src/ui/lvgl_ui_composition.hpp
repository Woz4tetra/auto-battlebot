#pragma once

#include <lvgl.h>

#include <memory>
#include <vector>

#include "lvgl_ui_services.hpp"
#include "lvgl_ui_widgets.hpp"

namespace auto_battlebot {
class UIState;
}

namespace auto_battlebot::ui_internal {

struct UiFrameState {
    lv_obj_t *tabview = nullptr;
    lv_image_dsc_t image_dsc = {};
    std::vector<uint8_t> image_copy;
    int diag_frame = 0;
};

struct UiServices {
    std::unique_ptr<IBatterySource> battery_source;
    std::unique_ptr<IDebugOverlayRenderer> debug_overlay_renderer;
};

void build_ui_content(lv_obj_t *root, UIWidgets &widgets, std::shared_ptr<UIState> ui_state,
                      UiFrameState &frame_state);
void update_ui_content(UIWidgets &widgets, std::shared_ptr<UIState> ui_state,
                       UiFrameState &frame_state, UiServices &services);

}  // namespace auto_battlebot::ui_internal
