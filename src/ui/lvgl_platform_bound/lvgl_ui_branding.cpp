#include "lvgl_platform_bound/lvgl_ui_branding.hpp"

#include <lvgl.h>

#include <cstddef>
#include <cstdint>

// Embedded at build time by cmake/bin2c.cmake from logo/ (see the
// embed_brand_asset() calls in CMakeLists.txt). Regenerate the source assets with
// logo/gen_splash_animation.py and logo/gen_logo_icon.py.
extern const uint8_t bwbots_splash_gif[];
extern const size_t bwbots_splash_gif_size;
extern const uint8_t bwbots_logo_png[];
extern const size_t bwbots_logo_png_size;

namespace auto_battlebot::ui_internal {
namespace {

// Splash timeline. The GIF plays once (53 frames at 25 ms plus a 500 ms hold on
// the assembled logo baked into the last frame's delay); LVGL fires
// LV_EVENT_READY when the animation completes, which starts the fade to the UI.
// The failsafe deletes the overlay if the gif never completes (e.g. decode
// error), so a splash problem can never block the app.
constexpr uint32_t kFadeMs = 350;
constexpr uint32_t kFailsafeMs = 8000;

lv_timer_t *g_failsafe_timer = nullptr;

const lv_image_dsc_t *raw_image_dsc(lv_image_dsc_t &dsc, const uint8_t *data, size_t size) {
    dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    dsc.header.cf = LV_COLOR_FORMAT_RAW_ALPHA;
    dsc.data = data;
    dsc.data_size = static_cast<uint32_t>(size);
    return &dsc;
}

void delete_overlay_cb(lv_timer_t *timer) {
    lv_obj_delete(static_cast<lv_obj_t *>(lv_timer_get_user_data(timer)));
}

// LV_EVENT_READY from the gif: the animation (including the final hold) finished
// and the last frame stays displayed. Fade the overlay out and delete it.
void splash_done_cb(lv_event_t *e) {
    lv_obj_t *gif = static_cast<lv_obj_t *>(lv_event_get_target(e));
    lv_obj_t *overlay = static_cast<lv_obj_t *>(lv_event_get_user_data(e));
    // Style opa does not propagate to children, so fade the gif alongside the overlay.
    lv_obj_fade_out(overlay, kFadeMs, 0);
    lv_obj_fade_out(gif, kFadeMs, 0);
    lv_timer_t *timer = lv_timer_create(delete_overlay_cb, kFadeMs + 50, overlay);
    lv_timer_set_repeat_count(timer, 1);
}

void failsafe_cb(lv_timer_t *timer) {
    g_failsafe_timer = nullptr;
    lv_obj_delete(static_cast<lv_obj_t *>(lv_timer_get_user_data(timer)));
}

void overlay_deleted_cb(lv_event_t *) {
    if (g_failsafe_timer != nullptr) {
        lv_timer_delete(g_failsafe_timer);
        g_failsafe_timer = nullptr;
    }
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
    lv_gif_set_loop_count(gif, 1);
    lv_obj_center(gif);

    lv_obj_add_event_cb(gif, splash_done_cb, LV_EVENT_READY, overlay);
    g_failsafe_timer = lv_timer_create(failsafe_cb, kFailsafeMs, overlay);
    lv_timer_set_repeat_count(g_failsafe_timer, 1);
    lv_obj_add_event_cb(overlay, overlay_deleted_cb, LV_EVENT_DELETE, nullptr);
}

const lv_image_dsc_t *bwbots_logo_icon() {
    static lv_image_dsc_t icon_dsc;
    return raw_image_dsc(icon_dsc, bwbots_logo_png, bwbots_logo_png_size);
}

}  // namespace auto_battlebot::ui_internal
