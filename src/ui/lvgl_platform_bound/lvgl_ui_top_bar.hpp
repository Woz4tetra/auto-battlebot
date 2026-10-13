#pragma once

#include <lvgl.h>

#include "lvgl_ui_widgets.hpp"

namespace auto_battlebot::ui_internal {

/** Logo, active profile, and clock along the top of the screen. */
void build_top_bar(lv_obj_t *parent, UIWidgets &w);
void update_top_bar(UIWidgets &w);

}  // namespace auto_battlebot::ui_internal
