#pragma once

#include <lvgl.h>

#include <string>

#include "lvgl_ui_services.hpp"
#include "lvgl_ui_widgets.hpp"

namespace auto_battlebot::ui_internal {

std::string normalize_battery_source(std::string source);
void build_top_bar(lv_obj_t *parent, UIWidgets &w);
void update_top_bar(UIWidgets &w, IBatterySource &battery_source);

}  // namespace auto_battlebot::ui_internal
