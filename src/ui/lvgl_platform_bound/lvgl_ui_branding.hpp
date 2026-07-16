#pragma once

#include <lvgl.h>

namespace auto_battlebot::ui_internal {

/// Plays the BW-bots splash animation full-screen on LVGL's top layer, then fades
/// out to reveal the live UI underneath. Purely cosmetic: the UI keeps updating
/// below the overlay, and input is absorbed until the overlay is deleted.
void show_splash_overlay();

/// Image descriptor for the small BW-bots logo (embedded PNG, decoded by lodepng).
const lv_image_dsc_t *bwbots_logo_icon();

}  // namespace auto_battlebot::ui_internal
