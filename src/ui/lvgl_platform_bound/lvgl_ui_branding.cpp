#include "lvgl_platform_bound/lvgl_ui_branding.hpp"

#include <lvgl.h>

#include <cstddef>
#include <cstdint>

// Embedded at build time by cmake/bin2c.cmake from scripts/assets/ (see the
// embed_brand_asset() calls in CMakeLists.txt). Regenerate the source assets with
// scripts/gen_splash_animation.py and scripts/gen_logo_icon.py.
extern const uint8_t bwbots_splash_gif[];
extern const size_t bwbots_splash_gif_size;
extern const uint8_t bwbots_logo_png[];
extern const size_t bwbots_logo_png_size;

namespace auto_battlebot::ui_internal {
namespace {

// Splash timeline. The GIF runs 53 frames at 25 ms plus a 500 ms hold on the
// assembled logo (~1.83 s total) and would then loop; it is paused during the
// hold window so the final logo stays up through the fade.
constexpr uint32_t kGifPauseMs = 1550;
constexpr uint32_t kFadeStartMs = 1850;
constexpr uint32_t kFadeMs = 350;

const lv_image_dsc_t *raw_image_dsc(lv_image_dsc_t &dsc, const uint8_t *data, size_t size) {
    dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    dsc.header.cf = LV_COLOR_FORMAT_RAW_ALPHA;
    dsc.data = data;
    dsc.data_size = static_cast<uint32_t>(size);
    return &dsc;
}

void pause_gif_cb(lv_timer_t *timer) {
    lv_gif_pause(static_cast<lv_obj_t *>(lv_timer_get_user_data(timer)));
}

void fade_out_cb(lv_timer_t *timer) {
    lv_obj_fade_out(static_cast<lv_obj_t *>(lv_timer_get_user_data(timer)), kFadeMs, 0);
}

void delete_overlay_cb(lv_timer_t *timer) {
    lv_obj_delete(static_cast<lv_obj_t *>(lv_timer_get_user_data(timer)));
}

void one_shot_timer(lv_timer_cb_t callback, uint32_t period_ms, void *user_data) {
    lv_timer_set_repeat_count(lv_timer_create(callback, period_ms, user_data), 1);
}

}  // namespace

void show_splash_overlay() {
    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(overlay, 0, 0);
    lv_obj_set_style_border_width(overlay, 0, 0);
    lv_obj_set_style_pad_all(overlay, 0, 0);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);

    static lv_image_dsc_t gif_dsc;
    lv_obj_t *gif = lv_gif_create(overlay);
    lv_gif_set_src(gif, raw_image_dsc(gif_dsc, bwbots_splash_gif, bwbots_splash_gif_size));
    lv_obj_center(gif);

    one_shot_timer(pause_gif_cb, kGifPauseMs, gif);
    one_shot_timer(fade_out_cb, kFadeStartMs, overlay);
    // Style opa does not propagate to children, so fade the gif alongside the overlay.
    one_shot_timer(fade_out_cb, kFadeStartMs, gif);
    one_shot_timer(delete_overlay_cb, kFadeStartMs + kFadeMs + 50, overlay);
}

const lv_image_dsc_t *bwbots_logo_icon() {
    static lv_image_dsc_t icon_dsc;
    return raw_image_dsc(icon_dsc, bwbots_logo_png, bwbots_logo_png_size);
}

}  // namespace auto_battlebot::ui_internal
