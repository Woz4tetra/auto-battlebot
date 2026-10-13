#include "lvgl_platform_bound/lvgl_ui_top_bar.hpp"

#include <ctime>

#include "lvgl_platform_bound/lvgl_ui_branding.hpp"

namespace auto_battlebot::ui_internal {

void build_top_bar(lv_obj_t *parent, UIWidgets &w) {
    lv_obj_t *bar = lv_obj_create(parent);
    w.top_bar = bar;
    lv_obj_set_width(bar, LV_PCT(100));
    lv_obj_set_height(bar, TOP_BAR_HEIGHT);
    lv_obj_set_style_pad_hor(bar, 12, 0);
    lv_obj_set_style_pad_ver(bar, 4, 0);
    lv_obj_set_style_pad_gap(bar, 8, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x111111), 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    /* BWBots logo, far left. Dark-background variant (black half drawn as a white
     * outline, matching the splash) with a transparent background. */
    lv_obj_t *logo = lv_image_create(bar);
    lv_image_set_src(logo, bwbots_logo_icon());
    lv_obj_align(logo, LV_ALIGN_LEFT_MID, 0, 0);

    /* Active config profile, right of the logo (text set from UIState in update_home). */
    w.profile_readout = lv_label_create(bar);
    lv_label_set_text(w.profile_readout, "--");
    lv_obj_set_style_text_font(w.profile_readout, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(w.profile_readout, lv_color_hex(0xB0B0B0), 0);
    lv_obj_align(w.profile_readout, LV_ALIGN_LEFT_MID, 40, 0);

    w.clock_label = lv_label_create(bar);
    lv_label_set_text(w.clock_label, "--:--:--");
    lv_obj_set_style_text_font(w.clock_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(w.clock_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_center(w.clock_label);
}

void update_top_bar(UIWidgets &w) {
    const std::time_t now_secs = std::time(nullptr);
    if (w.clock_label && now_secs != w.last_clock_second) {
        std::tm local_tm{};
        localtime_r(&now_secs, &local_tm);
        char time_buf[16];
        if (std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm) > 0) {
            lv_label_set_text(w.clock_label, time_buf);
        }
        w.last_clock_second = now_secs;
    }
}

}  // namespace auto_battlebot::ui_internal
