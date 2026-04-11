#include <lvgl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <vector>

#include "data_structures/target_selection.hpp"
#include "lvgl_platform_bound/lvgl_ui_battery.hpp"
#include "lvgl_platform_bound/lvgl_ui_composition.hpp"
#include "lvgl_platform_bound/lvgl_ui_presenter.hpp"
#include "lvgl_platform_bound/lvgl_ui_widgets.hpp"
#include "transform_utils.hpp"
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {
namespace {
using ui_internal::CAMERA_PANEL_WIDTH_PCT;
using ui_internal::CameraTouchData;
using ui_internal::DIAG_STALE_SEC;
using ui_internal::HealthRow;
using ui_internal::OpponentTileData;
using ui_internal::SystemActionTileData;
using ui_internal::TILE_PAD;
using ui_internal::TILE_RADIUS;
using ui_internal::UIWidgets;

void reinit_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (w && w->controller) w->controller->request_reinitialize();
}

void opp_cb(lv_event_t *e) {
    auto *d = static_cast<OpponentTileData *>(lv_event_get_user_data(e));
    if (d && d->widgets && d->widgets->controller) {
        d->widgets->controller->set_opponent_count(d->count);
        if (d->widgets) d->widgets->selected_opponent_count = d->count;
    }
}

void autonomy_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (!w || !w->controller) return;
    w->controller->toggle_autonomy();
}

void recording_toggle_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (w && w->controller) w->controller->toggle_recording();
}

void system_action_cb(lv_event_t *e) {
    auto *d = static_cast<SystemActionTileData *>(lv_event_get_user_data(e));
    if (!d || !d->widgets) return;
    d->widgets->pending_confirm_action = d->action;
    if (d->widgets->confirm_message) {
        const char *msg = "Confirm action?";
        if (d->action == UISystemAction::QUIT_APP) {
            msg = "Quit app now?\n(service will restart)";
        } else if (d->action == UISystemAction::REBOOT_HOST) {
            msg = "Reboot host now?";
        } else if (d->action == UISystemAction::POWEROFF_HOST) {
            msg = "Power off host now?";
        }
        lv_label_set_text(d->widgets->confirm_message, msg);
    }
    if (d->widgets->confirm_overlay)
        lv_obj_clear_flag(d->widgets->confirm_overlay, LV_OBJ_FLAG_HIDDEN);
}

void system_confirm_yes_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (!w || !w->confirm_overlay) return;
    UISystemAction action = w->pending_confirm_action;
    w->pending_confirm_action = UISystemAction::NONE;
    lv_obj_add_flag(w->confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    if (action == UISystemAction::NONE) return;
    switch (action) {
        case UISystemAction::QUIT_APP:
        case UISystemAction::REBOOT_HOST:
        case UISystemAction::POWEROFF_HOST:
            if (w->controller) w->controller->request_system_action(action);
            break;
        default:
            break;
    }
}

void system_confirm_no_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (!w || !w->confirm_overlay) return;
    w->pending_confirm_action = UISystemAction::NONE;
    lv_obj_add_flag(w->confirm_overlay, LV_OBJ_FLAG_HIDDEN);
}

void style_transparent(lv_obj_t *obj) {
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

void diagnostics_scroll_begin_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (!w) return;
    w->diag_user_scrolling = true;
}

void diagnostics_scroll_end_cb(lv_event_t *e) {
    auto *w = static_cast<UIWidgets *>(lv_event_get_user_data(e));
    if (!w) return;
    w->diag_user_scrolling = false;
}

HealthRow make_health_row(lv_obj_t *parent, const char *text) {
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(row, 4, 0);
    lv_obj_set_style_pad_gap(row, 12, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);

    lv_obj_t *led = lv_led_create(row);
    lv_obj_set_size(led, 20, 20);
    lv_led_set_color(led, lv_color_hex(0x00C853));
    lv_led_on(led);

    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);

    return {led, lbl};
}

void set_health(HealthRow &hr, bool ok, const char *text) {
    if (hr.led) lv_led_set_color(hr.led, ok ? lv_color_hex(0x00C853) : lv_color_hex(0xFF1744));
    if (hr.label) lv_label_set_text(hr.label, text);
}

void pulse_bg_opa_cb(void *var, int32_t v) {
    lv_obj_set_style_bg_opa(static_cast<lv_obj_t *>(var), static_cast<lv_opa_t>(v), 0);
}

void start_reinit_pulse(UIWidgets &w) {
    if (!w.reinit_tile || w.reinit_pulsing) return;
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, w.reinit_tile);
    lv_anim_set_values(&anim, static_cast<int32_t>(LV_OPA_60), static_cast<int32_t>(LV_OPA_COVER));
    lv_anim_set_duration(&anim, 650);
    lv_anim_set_reverse_duration(&anim, 650);
    lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_in_out);
    lv_anim_set_exec_cb(&anim, pulse_bg_opa_cb);
    lv_anim_start(&anim);
    w.reinit_pulsing = true;
}

void stop_reinit_pulse(UIWidgets &w) {
    if (!w.reinit_tile || !w.reinit_pulsing) return;
    lv_anim_delete(w.reinit_tile, pulse_bg_opa_cb);
    lv_obj_set_style_bg_opa(w.reinit_tile, LV_OPA_COVER, 0);
    w.reinit_pulsing = false;
}

void set_manual_target_ui_state(UIWidgets &w, bool active) {
    w.manual_target_active = active;
    if (w.camera_frame) {
        // Keep border width constant so image geometry does not shift while pressing.
        lv_obj_set_style_border_width(w.camera_frame, 2, 0);
        lv_obj_set_style_border_color(w.camera_frame,
                                      active ? lv_color_hex(0xFFC107) : lv_color_hex(0x424242), 0);
    }
    if (w.manual_target_label) {
        if (active)
            lv_obj_clear_flag(w.manual_target_label, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(w.manual_target_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (w.opp_label) {
        lv_obj_set_style_text_color(w.opp_label,
                                    active ? lv_color_hex(0x9E9E9E) : lv_color_hex(0xFFFFFF), 0);
    }
    for (int i = 0; i < 3; i++) {
        if (!w.opp_tiles[i]) continue;
        if (active) {
            lv_obj_add_state(w.opp_tiles[i], LV_STATE_DISABLED);
            lv_obj_clear_flag(w.opp_tiles[i], LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_remove_state(w.opp_tiles[i], LV_STATE_DISABLED);
            lv_obj_add_flag(w.opp_tiles[i], LV_OBJ_FLAG_CLICKABLE);
        }
    }
}

bool event_to_image_pixel(const UIWidgets &w, lv_event_t *e, int &img_x, int &img_y) {
    if (!w.debug_img || w.camera_src_width <= 0 || w.camera_src_height <= 0) return false;
    lv_indev_t *indev = lv_event_get_indev(e);
    if (!indev) return false;
    lv_point_t p{};
    lv_indev_get_point(indev, &p);

    lv_area_t img_area{};
    lv_obj_get_coords(w.debug_img, &img_area);
    if (p.x < img_area.x1 || p.x > img_area.x2 || p.y < img_area.y1 || p.y > img_area.y2) {
        return false;
    }

    const int32_t draw_w = lv_obj_get_width(w.debug_img);
    const int32_t draw_h = lv_obj_get_height(w.debug_img);
    if (draw_w <= 1 || draw_h <= 1) return false;

    const int32_t rel_x = p.x - img_area.x1;
    const int32_t rel_y = p.y - img_area.y1;

    img_x =
        static_cast<int>(std::lround((static_cast<double>(rel_x) * w.camera_src_width) / draw_w));
    img_y =
        static_cast<int>(std::lround((static_cast<double>(rel_y) * w.camera_src_height) / draw_h));
    img_x = std::clamp(img_x, 0, w.camera_src_width - 1);
    img_y = std::clamp(img_y, 0, w.camera_src_height - 1);
    return true;
}

bool project_image_pixel_to_field(const FieldDescription &field, const CameraInfo &camera_info,
                                  int source_img_w, int source_img_h, int img_x, int img_y,
                                  Pose2D &out) {
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) return false;

    double pixel_x = static_cast<double>(img_x);
    double pixel_y = static_cast<double>(img_y);
    if (source_img_w > 0 && source_img_h > 0 && camera_info.width > 0 && camera_info.height > 0 &&
        (source_img_w != camera_info.width || source_img_h != camera_info.height)) {
        const double sx = static_cast<double>(camera_info.width) / source_img_w;
        const double sy = static_cast<double>(camera_info.height) / source_img_h;
        pixel_x *= sx;
        pixel_y *= sy;
    }
    Eigen::Vector3d ray;
    if (!pixel_to_camera_ray(camera_info, pixel_x, pixel_y, ray)) return false;

    Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
    if (!transform_to_plane_center_normal(field.tf_camera_from_fieldcenter, plane_center,
                                          plane_normal))
        return false;

    Eigen::Vector3d camera_pt;
    if (!intersect_camera_ray_with_plane(ray, plane_center, plane_normal, camera_pt)) return false;
    const Eigen::Vector4d camera_h(camera_pt.x(), camera_pt.y(), camera_pt.z(), 1.0);
    const Eigen::Vector4d field_h = field.tf_camera_from_fieldcenter.tf.inverse() * camera_h;
    if (!std::isfinite(field_h.x()) || !std::isfinite(field_h.y())) return false;

    out.x = field_h.x();
    out.y = field_h.y();
    out.yaw = 0.0;
    return true;
}

void camera_touch_cb(lv_event_t *e) {
    auto *d = static_cast<CameraTouchData *>(lv_event_get_user_data(e));
    if (!d || !d->widgets || !d->widgets->ui_state) return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (d->widgets->controller) d->widgets->controller->set_manual_target(std::nullopt);
        set_manual_target_ui_state(*d->widgets, false);
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;

    int img_x = 0, img_y = 0;
    if (!event_to_image_pixel(*d->widgets, e, img_x, img_y)) return;

    std::optional<FieldDescription> field;
    d->widgets->ui_state->get_field_description(field);
    CameraInfo camera_info;
    d->widgets->ui_state->get_camera_info(camera_info);
    if (!field.has_value()) return;

    Pose2D target_pose_field;
    if (!project_image_pixel_to_field(*field, camera_info, d->widgets->camera_src_width,
                                      d->widgets->camera_src_height, img_x, img_y,
                                      target_pose_field))
        return;
    TargetSelection target{target_pose_field, Label::EMPTY};
    if (d->widgets->controller) d->widgets->controller->set_manual_target(target);
    set_manual_target_ui_state(*d->widgets, true);
}

/* ------------------------------------------------------------------ */
/*  Home tab                                                          */
/* ------------------------------------------------------------------ */

void build_home(lv_obj_t *tab, UIWidgets &w) {
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(tab, 8, 0);
    lv_obj_set_style_pad_gap(tab, 8, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *top_row = lv_obj_create(tab);
    lv_obj_set_width(top_row, LV_PCT(100));
    lv_obj_set_flex_grow(top_row, 3);
    lv_obj_set_flex_flow(top_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(top_row, 0, 0);
    lv_obj_set_style_pad_gap(top_row, 8, 0);
    style_transparent(top_row);

    lv_obj_t *controls = lv_obj_create(top_row);
    lv_obj_set_width(controls, LV_PCT(100 - CAMERA_PANEL_WIDTH_PCT));
    lv_obj_set_height(controls, LV_PCT(100));
    lv_obj_set_flex_flow(controls, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(controls, 0, 0);
    lv_obj_set_style_pad_gap(controls, 8, 0);
    style_transparent(controls);

    lv_obj_t *camera_col = lv_obj_create(top_row);
    lv_obj_set_width(camera_col, LV_PCT(CAMERA_PANEL_WIDTH_PCT));
    lv_obj_set_height(camera_col, LV_PCT(100));
    lv_obj_set_flex_flow(camera_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(camera_col, 0, 0);
    lv_obj_clear_flag(camera_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(camera_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(camera_col, 0, 0);

    lv_obj_t *top = lv_obj_create(controls);
    lv_obj_set_width(top, LV_PCT(100));
    lv_obj_set_height(top, LV_PCT(100));
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(top, 0, 0);
    lv_obj_set_style_pad_gap(top, 8, 0);
    style_transparent(top);

    /* Status tile: whole tile changes color (yellow = system OK, green = OK +
     * tracking) */
    lv_obj_t *st = lv_obj_create(top);
    w.status_tile = st;
    lv_obj_set_width(st, LV_PCT(100));
    lv_obj_set_flex_grow(st, 1);
    lv_obj_set_style_radius(st, TILE_RADIUS, 0);
    lv_obj_set_style_pad_all(st, TILE_PAD, 0);
    lv_obj_set_flex_flow(st, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(st, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(st, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(st, lv_color_hex(0x757575), 0); /* default gray until status known */

    w.status_label = lv_label_create(st);
    lv_label_set_text(w.status_label, "Waiting...");
    lv_obj_set_style_text_font(w.status_label, &lv_font_montserrat_28, 0);

    w.status_detail = lv_label_create(st);
    lv_label_set_text(w.status_detail, "");
    lv_obj_set_style_text_font(w.status_detail, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(w.status_detail, lv_color_hex(0xAAAAAA), 0);

    /* Autonomy toggle tile */
    lv_obj_t *at = lv_obj_create(top);
    w.autonomy_tile = at;
    lv_obj_set_width(at, LV_PCT(100));
    lv_obj_set_flex_grow(at, 1);
    lv_obj_set_style_radius(at, TILE_RADIUS, 0);
    lv_obj_set_style_pad_all(at, TILE_PAD, 0);
    lv_obj_set_flex_flow(at, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(at, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(at, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(at, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(at, lv_color_hex(0x757575), 0);
    lv_obj_set_style_bg_color(at, lv_color_hex(0xBDBDBD), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(at, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_add_event_cb(at, autonomy_cb, LV_EVENT_CLICKED, &w);

    w.autonomy_label = lv_label_create(at);
    lv_label_set_text(w.autonomy_label, "Autonomy\n--");
    lv_obj_set_style_text_font(w.autonomy_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(w.autonomy_label, LV_TEXT_ALIGN_CENTER, 0);

    /* Reinit tile */
    lv_obj_t *rt = lv_obj_create(top);
    w.reinit_tile = rt;
    lv_obj_set_width(rt, LV_PCT(100));
    lv_obj_set_flex_grow(rt, 1);
    lv_obj_set_style_radius(rt, TILE_RADIUS, 0);
    lv_obj_set_style_pad_all(rt, TILE_PAD, 0);
    lv_obj_set_flex_flow(rt, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(rt, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(rt, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(rt, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(rt, lv_color_hex(0xFF1744), 0); /* red until field found */
    lv_obj_set_style_bg_color(rt, lv_color_hex(0xBDBDBD),
                              LV_STATE_PRESSED); /* gray flash on press */
    lv_obj_set_style_bg_opa(rt, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_add_event_cb(rt, reinit_cb, LV_EVENT_CLICKED, &w);

    lv_obj_t *ri = lv_label_create(rt);
    lv_label_set_text(ri, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_font(ri, &lv_font_montserrat_32, 0);

    lv_obj_t *rl = lv_label_create(rt);
    lv_label_set_text(rl, "Reinitialize Field");
    lv_obj_set_style_text_font(rl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(rl, LV_TEXT_ALIGN_CENTER, 0);

    w.reinit_status = lv_label_create(rt);
    lv_label_set_text(w.reinit_status, "");
    lv_obj_set_style_text_font(w.reinit_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(w.reinit_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(w.reinit_status, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t *opp_section = lv_obj_create(tab);
    lv_obj_set_width(opp_section, LV_PCT(100));
    lv_obj_set_flex_grow(opp_section, 1);
    lv_obj_set_flex_flow(opp_section, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(opp_section, 0, 0);
    lv_obj_set_style_pad_gap(opp_section, 6, 0);
    style_transparent(opp_section);

    w.opp_label = lv_label_create(opp_section);
    lv_label_set_text(w.opp_label, "Target Opponents");
    lv_obj_set_style_text_font(w.opp_label, &lv_font_montserrat_20, 0);

    w.manual_target_label = lv_label_create(opp_section);
    lv_label_set_text(w.manual_target_label, "manual targeting");
    lv_obj_set_style_text_font(w.manual_target_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(w.manual_target_label, lv_color_hex(0xFFC107), 0);
    lv_obj_add_flag(w.manual_target_label, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *bot = lv_obj_create(opp_section);
    lv_obj_set_width(bot, LV_PCT(100));
    lv_obj_set_flex_grow(bot, 1);
    lv_obj_set_flex_flow(bot, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(bot, 0, 0);
    lv_obj_set_style_pad_gap(bot, 8, 0);
    style_transparent(bot);

    for (int i = 0; i < 3; i++) {
        w.opp_tile_data[i] = {&w, i + 1};

        lv_obj_t *tile = lv_obj_create(bot);
        lv_obj_set_flex_grow(tile, 1);
        lv_obj_set_height(tile, LV_PCT(100));
        lv_obj_set_style_radius(tile, TILE_RADIUS, 0);
        lv_obj_set_style_pad_all(tile, TILE_PAD, 0);
        lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(tile, opp_cb, LV_EVENT_CLICKED, &w.opp_tile_data[i]);

        lv_obj_set_style_border_color(tile, lv_color_hex(0x2979FF), LV_STATE_CHECKED);
        lv_obj_set_style_border_width(tile, 4, LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(tile, lv_color_hex(0x1A237E), LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(tile, lv_color_hex(0x424242), LV_STATE_DISABLED);
        lv_obj_set_style_border_color(tile, lv_color_hex(0x616161), LV_STATE_DISABLED);
        lv_obj_set_style_opa(tile, LV_OPA_50, LV_STATE_DISABLED);

        lv_obj_t *num = lv_label_create(tile);
        lv_label_set_text_fmt(num, "%d", i + 1);
        lv_obj_set_style_text_font(num, &lv_font_montserrat_32, 0);

        lv_obj_t *desc = lv_label_create(tile);
        lv_label_set_text(desc, (i == 0) ? "Opponent" : "Opponents");
        lv_obj_set_style_text_font(desc, &lv_font_montserrat_20, 0);

        w.opp_tiles[i] = tile;
    }
    w.camera_frame = lv_obj_create(camera_col);
    lv_obj_set_size(w.camera_frame, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(w.camera_frame, 4, 0);
    lv_obj_set_style_radius(w.camera_frame, TILE_RADIUS, 0);
    lv_obj_set_style_border_width(w.camera_frame, 2, 0);
    lv_obj_set_style_border_color(w.camera_frame, lv_color_hex(0x424242), 0);
    lv_obj_clear_flag(w.camera_frame, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(w.camera_frame, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(w.camera_frame, LV_OBJ_FLAG_EVENT_BUBBLE);

    w.camera_touch = lv_obj_create(w.camera_frame);
    lv_obj_set_size(w.camera_touch, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(w.camera_touch, 0, 0);
    lv_obj_set_style_radius(w.camera_touch, 0, 0);
    lv_obj_set_style_bg_opa(w.camera_touch, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(w.camera_touch, 0, 0);
    lv_obj_clear_flag(w.camera_touch, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(w.camera_touch, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(w.camera_touch, LV_OBJ_FLAG_EVENT_BUBBLE);

    w.camera_touch_data.widgets = &w;
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_PRESSED, &w.camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_PRESSING, &w.camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_RELEASED, &w.camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_PRESS_LOST, &w.camera_touch_data);

    w.debug_img = lv_image_create(w.camera_touch);
    lv_obj_align(w.debug_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(w.debug_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(w.debug_img, LV_OBJ_FLAG_EVENT_BUBBLE);

    w.debug_kp_cont = lv_obj_create(w.camera_touch);
    lv_obj_set_size(w.debug_kp_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_align(w.debug_kp_cont, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(w.debug_kp_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(w.debug_kp_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(w.debug_kp_cont, 0, 0);
    lv_obj_set_style_pad_all(w.debug_kp_cont, 0, 0);
    lv_obj_set_style_radius(w.debug_kp_cont, 0, 0);
    lv_obj_add_flag(w.debug_kp_cont, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(w.debug_kp_cont, LV_OBJ_FLAG_EVENT_BUBBLE);

    w.kp_dots.clear();
    set_manual_target_ui_state(w, false);
}

/* ------------------------------------------------------------------ */
/*  Diagnostics tab                                                   */
/* ------------------------------------------------------------------ */

void build_diagnostics(lv_obj_t *tab, UIWidgets &w) {
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(tab, 8, 0);
    lv_obj_set_style_pad_gap(tab, 8, 0);
    /* Single scroll for the whole diagnostics page */
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_scroll_dir(tab, LV_DIR_VER);
    lv_obj_add_event_cb(tab, diagnostics_scroll_begin_cb, LV_EVENT_SCROLL_BEGIN, &w);
    lv_obj_add_event_cb(tab, diagnostics_scroll_end_cb, LV_EVENT_SCROLL_END, &w);

    lv_obj_t *hdr = lv_label_create(tab);
    lv_label_set_text(hdr, "System Health");
    lv_obj_set_style_text_font(hdr, &lv_font_montserrat_24, 0);

    lv_obj_t *hc = lv_obj_create(tab);
    lv_obj_set_size(hc, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hc, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(hc, 8, 0);
    lv_obj_set_style_pad_gap(hc, 2, 0);
    lv_obj_clear_flag(hc, LV_OBJ_FLAG_SCROLLABLE);

    w.health[0] = make_health_row(hc, "Camera: --");
    w.health[1] = make_health_row(hc, "Transmitter: --");
    w.health[2] = make_health_row(hc, "Field: --");
    w.health[3] = make_health_row(hc, "Our Robot Seen: --");
    w.health[4] = make_health_row(hc, "Opponents Seen: --");
    w.health[5] = make_health_row(hc, "Loop Rate: --");
    w.health[6] = make_health_row(hc, "Jetson Temp: --");
    w.health[7] = make_health_row(hc, "Compute Mode: --");
    w.health_count = 8;

    lv_obj_t *dh = lv_label_create(tab);
    lv_label_set_text(dh, "Diagnostic Details");
    lv_obj_set_style_text_font(dh, &lv_font_montserrat_24, 0);

    w.sections_cont = lv_obj_create(tab);
    lv_obj_set_width(w.sections_cont, LV_PCT(100));
    lv_obj_set_height(w.sections_cont, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(w.sections_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(w.sections_cont, 4, 0);
    lv_obj_set_style_pad_gap(w.sections_cont, 4, 0);
    lv_obj_clear_flag(w.sections_cont, LV_OBJ_FLAG_SCROLLABLE);
}

/* ------------------------------------------------------------------ */
/*  System tab                                                        */
/* ------------------------------------------------------------------ */

void build_system(lv_obj_t *tab, UIWidgets &w, std::shared_ptr<UIState> ui_state) {
    w.ui_state = ui_state;
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(tab, 8, 0);
    lv_obj_set_style_pad_gap(tab, 8, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *recording_tile = lv_obj_create(tab);
    w.recording_tile = recording_tile;
    lv_obj_set_width(recording_tile, LV_PCT(100));
    lv_obj_set_flex_grow(recording_tile, 1);
    lv_obj_set_style_radius(recording_tile, TILE_RADIUS, 0);
    lv_obj_set_style_pad_all(recording_tile, TILE_PAD, 0);
    lv_obj_set_flex_flow(recording_tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(recording_tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(recording_tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(recording_tile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(recording_tile, lv_color_hex(0x757575), 0);
    lv_obj_set_style_bg_color(recording_tile, lv_color_hex(0xBDBDBD), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(recording_tile, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_add_event_cb(recording_tile, recording_toggle_cb, LV_EVENT_CLICKED, &w);

    w.recording_label = lv_label_create(recording_tile);
    lv_label_set_text(w.recording_label, "Recording\n--");
    lv_obj_set_style_text_font(w.recording_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(w.recording_label, LV_TEXT_ALIGN_CENTER, 0);

    w.recording_detail = lv_label_create(recording_tile);
    lv_label_set_text(w.recording_detail, "SVO -- | MCAP --");
    lv_obj_set_style_text_font(w.recording_detail, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_align(w.recording_detail, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *actions_row = lv_obj_create(tab);
    lv_obj_set_width(actions_row, LV_PCT(100));
    lv_obj_set_flex_grow(actions_row, 1);
    lv_obj_set_style_pad_all(actions_row, 0, 0);
    lv_obj_set_style_pad_gap(actions_row, 8, 0);
    lv_obj_set_flex_flow(actions_row, LV_FLEX_FLOW_ROW);
    style_transparent(actions_row);

    struct ActionSpec {
        const char *title;
        const char *subtitle;
        lv_color_t color;
        UISystemAction action;
        lv_obj_t **widget_slot;
    };
    std::array<ActionSpec, 3> specs{{
        {"Quit App", "service restarts", lv_color_hex(0xFB8C00), UISystemAction::QUIT_APP,
         &w.quit_tile},
        {"Reboot Host", "full system reboot", lv_color_hex(0x1E88E5), UISystemAction::REBOOT_HOST,
         &w.reboot_tile},
        {"Power Off", "safe shutdown", lv_color_hex(0xE53935), UISystemAction::POWEROFF_HOST,
         &w.poweroff_tile},
    }};

    for (size_t i = 0; i < specs.size(); ++i) {
        const auto &spec = specs[i];
        w.system_action_tile_data[i] = {&w, spec.action};

        lv_obj_t *tile = lv_obj_create(actions_row);
        *spec.widget_slot = tile;
        lv_obj_set_flex_grow(tile, 1);
        lv_obj_set_height(tile, LV_PCT(100));
        lv_obj_set_style_radius(tile, TILE_RADIUS, 0);
        lv_obj_set_style_pad_all(tile, TILE_PAD, 0);
        lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(tile, spec.color, 0);
        lv_obj_set_style_bg_color(tile, lv_color_hex(0xBDBDBD), LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, LV_STATE_PRESSED);
        lv_obj_add_event_cb(tile, system_action_cb, LV_EVENT_CLICKED,
                            &w.system_action_tile_data[i]);

        lv_obj_t *title = lv_label_create(tile);
        lv_label_set_text(title, spec.title);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

        lv_obj_t *subtitle = lv_label_create(tile);
        lv_label_set_text(subtitle, spec.subtitle);
        lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(subtitle, LV_TEXT_ALIGN_CENTER, 0);
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    w.confirm_overlay = overlay;
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_50, 0);
    lv_obj_set_style_border_width(overlay, 0, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *panel = lv_obj_create(overlay);
    lv_obj_set_size(panel, 360, 180);
    lv_obj_center(panel);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(panel, TILE_RADIUS, 0);
    lv_obj_set_style_pad_all(panel, TILE_PAD, 0);
    lv_obj_set_style_pad_gap(panel, 12, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    w.confirm_message = lv_label_create(panel);
    lv_label_set_text(w.confirm_message, "Confirm action?");
    lv_obj_set_style_text_font(w.confirm_message, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_align(w.confirm_message, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *btn_row = lv_obj_create(panel);
    lv_obj_set_width(btn_row, LV_PCT(100));
    lv_obj_set_height(btn_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_style_pad_gap(btn_row, 10, 0);
    style_transparent(btn_row);

    lv_obj_t *cancel_btn = lv_btn_create(btn_row);
    lv_obj_set_flex_grow(cancel_btn, 1);
    lv_obj_set_height(cancel_btn, 48);
    lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x616161), 0);
    lv_obj_add_event_cb(cancel_btn, system_confirm_no_cb, LV_EVENT_CLICKED, &w);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);

    lv_obj_t *confirm_btn = lv_btn_create(btn_row);
    lv_obj_set_flex_grow(confirm_btn, 1);
    lv_obj_set_height(confirm_btn, 48);
    lv_obj_set_style_bg_color(confirm_btn, lv_color_hex(0xE53935), 0);
    lv_obj_add_event_cb(confirm_btn, system_confirm_yes_cb, LV_EVENT_CLICKED, &w);
    lv_obj_t *confirm_lbl = lv_label_create(confirm_btn);
    lv_label_set_text(confirm_lbl, "Confirm");
    lv_obj_center(confirm_lbl);
}

/* ------------------------------------------------------------------ */
/*  Update functions                                                  */
/* ------------------------------------------------------------------ */

void update_home(UIWidgets &w, std::shared_ptr<UIState> us) {
    if (!us) return;

    SystemStatus st;
    us->get_system_status(st);
    RobotDescriptionsStamped robots;
    us->get_robots(robots);
    bool our_seen = false;
    int opp = 0;
    ui_internal::derive_robot_counts(robots, our_seen, opp);

    ui_internal::update_rate_history(w.rate_history, us->get_rate_avg_window(), st.loop_rate_hz);
    const double avg_hz = ui_internal::get_rate_avg(w.rate_history, st.loop_rate_hz);
    const bool loop_met = ui_internal::compute_loop_met_sustained(
        w.rate_below_since, avg_hz, us->get_max_loop_rate(), us->get_rate_fail_threshold(),
        us->get_rate_fail_duration_sec());
    const ui_internal::HomeViewModel vm =
        ui_internal::present_home(st, our_seen, opp, w.selected_opponent_count, avg_hz, loop_met);
    w.selected_opponent_count = vm.selected_opponent_count;

    if (w.status_tile) {
        lv_obj_set_style_bg_color(w.status_tile, lv_color_hex(vm.status_tile_color), 0);
    }
    if (w.status_label) {
        lv_label_set_text(w.status_label, vm.status_label_text.c_str());
        lv_obj_set_style_text_color(w.status_label, lv_color_hex(vm.status_label_text_color), 0);
    }

    if (w.status_detail) {
        lv_obj_set_style_text_color(w.status_detail, lv_color_hex(vm.status_detail_text_color), 0);
        lv_label_set_text(w.status_detail, vm.status_detail_text.c_str());
    }

    if (w.reinit_tile) {
        lv_obj_set_style_bg_color(w.reinit_tile, lv_color_hex(vm.reinit_tile_color), 0);
    }
    if (w.reinit_status) {
        lv_label_set_text(w.reinit_status, vm.reinit_status_text.c_str());
    }

    if (vm.reinit_should_pulse) {
        start_reinit_pulse(w);
    } else {
        stop_reinit_pulse(w);
    }

    if (w.autonomy_tile) {
        lv_obj_set_style_bg_color(w.autonomy_tile, lv_color_hex(vm.autonomy_tile_color), 0);
    }
    if (w.autonomy_label) {
        lv_label_set_text(w.autonomy_label, vm.autonomy_label_text.c_str());
        lv_obj_set_style_text_color(w.autonomy_label, lv_color_hex(vm.autonomy_label_text_color),
                                    0);
    }

    for (int i = 0; i < 3; i++) {
        if (!w.opp_tiles[i]) continue;
        if (w.selected_opponent_count == i + 1)
            lv_obj_add_state(w.opp_tiles[i], LV_STATE_CHECKED);
        else
            lv_obj_remove_state(w.opp_tiles[i], LV_STATE_CHECKED);
    }
}

void update_system(UIWidgets &w, std::shared_ptr<UIState> us) {
    if (!us) return;
    SystemStatus st;
    us->get_system_status(st);
    const ui_internal::SystemViewModel vm = ui_internal::present_system(st);

    if (w.recording_tile) {
        lv_obj_set_style_bg_color(w.recording_tile, lv_color_hex(vm.recording_tile_color), 0);
    }
    if (w.recording_label) {
        lv_label_set_text(w.recording_label, vm.recording_label_text.c_str());
        lv_obj_set_style_text_color(w.recording_label, lv_color_hex(vm.recording_label_text_color),
                                    0);
    }
    if (w.recording_detail) {
        lv_label_set_text(w.recording_detail, vm.recording_detail_text.c_str());
        lv_obj_set_style_text_color(w.recording_detail,
                                    lv_color_hex(vm.recording_detail_text_color), 0);
    }
}

void update_health_rows(UIWidgets &w, std::shared_ptr<UIState> us) {
    if (!us) return;

    SystemStatus st;
    us->get_system_status(st);
    RobotDescriptionsStamped robots;
    us->get_robots(robots);
    bool our_seen = false;
    int opp = 0;
    ui_internal::derive_robot_counts(robots, our_seen, opp);
    const double avg_hz = ui_internal::get_rate_avg(w.rate_history, st.loop_rate_hz);
    const bool loop_met = ui_internal::compute_loop_met_sustained(
        w.rate_below_since, avg_hz, us->get_max_loop_rate(), us->get_rate_fail_threshold(),
        us->get_rate_fail_duration_sec());
    const std::vector<ui_internal::HealthRowViewModel> rows =
        ui_internal::present_health(st, our_seen, opp, avg_hz, loop_met);
    const size_t limit = std::min(rows.size(), static_cast<size_t>(8));
    for (size_t i = 0; i < limit; ++i) {
        if (!rows[i].visible) continue;
        set_health(w.health[i], rows[i].ok, rows[i].text.c_str());
    }
}

void rebuild_diag_sections(UIWidgets &w, std::shared_ptr<UIState> us) {
    if (!us || !w.sections_cont) return;

    const auto now = std::chrono::steady_clock::now();

    /* Merge new snapshots into cache; keep old entries, never remove */
    std::vector<DiagnosticStatusSnapshot> snaps;
    us->get_diagnostic_snapshots(snaps);
    ui_internal::merge_diag_snapshots(w.diag_section_order, w.diag_section_cache, snaps, now);

    if (w.diag_user_scrolling) {
        w.diag_rebuild_pending = true;
        return;
    }

    /* Preserve scroll position when rebuilding content */
    lv_obj_t *diag_tab = lv_obj_get_parent(w.sections_cont);
    const int32_t scroll_y = (diag_tab != nullptr) ? lv_obj_get_scroll_top(diag_tab) : 0;

    lv_obj_clean(w.sections_cont);
    const std::vector<ui_internal::DiagnosticsSectionViewModel> sections =
        ui_internal::present_diagnostics_sections(w.diag_section_order, w.diag_section_cache, now,
                                                  DIAG_STALE_SEC);
    for (const auto &section : sections) {
        /* Section header bar: green when fresh, gray when stale */
        lv_obj_t *hdr = lv_obj_create(w.sections_cont);
        lv_obj_set_size(hdr, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(
            hdr, section.stale ? lv_color_hex(0x757575) : lv_color_hex(0x4CAF50), 0);
        lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(hdr, 4, 0);
        lv_obj_set_style_radius(hdr, 4, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *hlbl = lv_label_create(hdr);
        lv_label_set_text(hlbl, section.title.c_str());
        lv_obj_set_style_text_font(hlbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(hlbl, lv_color_hex(0xFFFFFF), 0);

        size_t rows = section.rows.size();
        if (rows == 0) continue;

        lv_obj_t *table = lv_table_create(w.sections_cont);
        lv_table_set_column_count(table, 2);
        lv_table_set_row_count(table, static_cast<uint32_t>(rows));
        lv_table_set_column_width(table, 0, 320);
        lv_table_set_column_width(table, 1, 480);
        lv_obj_set_width(table, LV_PCT(100));
        lv_obj_set_style_text_font(table, &lv_font_montserrat_20, LV_PART_ITEMS);
        lv_obj_set_style_pad_all(table, 2, LV_PART_ITEMS);
        if (section.stale)
            lv_obj_set_style_text_color(table, lv_color_hex(0x9E9E9E), LV_PART_ITEMS);

        uint32_t row = 0;
        for (const auto &[k, v] : section.rows) {
            lv_table_set_cell_value(table, row, 0, k.c_str());
            lv_table_set_cell_value(table, row, 1, v.c_str());
            row++;
        }
    }

    w.diag_rebuild_pending = false;
    if (diag_tab != nullptr) lv_obj_scroll_to_y(diag_tab, scroll_y, LV_ANIM_OFF);
}

void update_debug(UIWidgets &w, std::shared_ptr<UIState> us, lv_image_dsc_t &img_copy_dsc,
                  std::vector<uint8_t> &img_copy,
                  ui_internal::IDebugOverlayRenderer *overlay_renderer) {
    if (!w.debug_img || !us) return;

    int dw = 0, dh = 0, dc = 0;
    std::vector<uint8_t> data;
    us->get_debug_image(dw, dh, dc, data);
    RobotDescriptionsStamped robots;
    us->get_robots(robots);
    std::optional<NavigationPathSegment> navigation_path;
    us->get_navigation_path(navigation_path);
    std::optional<FieldDescription> field_description;
    us->get_field_description(field_description);
    CameraInfo camera_info;
    us->get_camera_info(camera_info);

    if (dw > 0 && dh > 0 && !data.empty()) {
        size_t expected = static_cast<size_t>(dw) * dh * (dc == 4 ? 4 : 3);
        if (data.size() >= expected) {
            std::vector<uint8_t> src_copy = data;
            if (field_description.has_value()) {
                if (camera_info.width <= 0 || camera_info.height <= 0) {
                    camera_info.width = dw;
                    camera_info.height = dh;
                }
                cv::Mat image(dh, dw, dc == 4 ? CV_8UC4 : CV_8UC3, src_copy.data());
                if (overlay_renderer) {
                    overlay_renderer->render(image, robots, navigation_path, *field_description,
                                             camera_info);
                }
            }
            w.camera_src_width = dw;
            w.camera_src_height = dh;

            /* Resize image buffer to fit container while preserving full frame. */
            lv_obj_t *cont = lv_obj_get_parent(w.debug_img);
            const int32_t cont_w = cont ? lv_obj_get_content_width(cont) : dw;
            const int32_t cont_h = cont ? lv_obj_get_content_height(cont) : dh;
            const double scale = std::min(
                1.0, std::min(static_cast<double>(cont_w) / dw, static_cast<double>(cont_h) / dh));
            const int32_t sw = std::max(1, static_cast<int32_t>(std::round(dw * scale)));
            const int32_t sh = std::max(1, static_cast<int32_t>(std::round(dh * scale)));

            const int channels = (dc == 4) ? 4 : 3;
            if (sw == dw && sh == dh) {
                img_copy = std::move(src_copy);
            } else {
                img_copy.resize(static_cast<size_t>(sw) * sh * channels);
                cv::Mat src_image(dh, dw, channels == 4 ? CV_8UC4 : CV_8UC3, src_copy.data());
                cv::Mat dst_image(sh, sw, channels == 4 ? CV_8UC4 : CV_8UC3, img_copy.data());
                cv::resize(src_image, dst_image, dst_image.size(), 0.0, 0.0, cv::INTER_LINEAR);
            }

            img_copy_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
            img_copy_dsc.header.cf = (dc == 4) ? LV_COLOR_FORMAT_XRGB8888 : LV_COLOR_FORMAT_RGB888;
            img_copy_dsc.header.flags = 0;
            img_copy_dsc.header.w = static_cast<uint32_t>(sw);
            img_copy_dsc.header.h = static_cast<uint32_t>(sh);
            img_copy_dsc.header.stride = sw * channels;
            img_copy_dsc.data_size = static_cast<uint32_t>(img_copy.size());
            img_copy_dsc.data = img_copy.data();
            lv_image_set_src(w.debug_img, &img_copy_dsc);

            lv_obj_set_size(w.debug_img, sw, sh);
            lv_obj_set_size(w.debug_kp_cont, sw, sh);
            lv_obj_align(w.debug_img, LV_ALIGN_CENTER, 0, 0);
            lv_obj_align(w.debug_kp_cont, LV_ALIGN_CENTER, 0, 0);
        }
    }

    for (lv_obj_t *dot : w.kp_dots) {
        if (dot) lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
    }
}

}  // namespace

namespace ui_internal {
void build_ui_content(lv_obj_t *root, UIWidgets &widgets, std::shared_ptr<UIState> ui_state,
                      UiFrameState &frame_state) {
    widgets.ui_state = ui_state;
    widgets.controller = std::make_shared<UiController>(ui_state);
    build_top_bar(root, widgets);

    lv_obj_t *tabview = lv_tabview_create(root);
    lv_obj_set_width(tabview, LV_PCT(100));
    lv_obj_set_flex_grow(tabview, 1);
    lv_obj_set_style_pad_all(tabview, 0, 0);
    lv_tabview_set_tab_bar_size(tabview, TAB_BAR_HEIGHT);
    lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *tab_content = lv_tabview_get_content(tabview);
    if (tab_content) {
        lv_obj_clear_flag(tab_content, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(tab_content, LV_DIR_NONE);
    }

    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(tabview);
    if (tab_bar) lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_24, 0);

    lv_obj_t *tab_home = lv_tabview_add_tab(tabview, "Home");
    lv_obj_t *tab_diag = lv_tabview_add_tab(tabview, "Diagnostics");
    lv_obj_t *tab_system = lv_tabview_add_tab(tabview, "System");

    widgets.tabview = tabview;
    frame_state.tabview = tabview;
    build_home(tab_home, widgets);
    build_diagnostics(tab_diag, widgets);
    build_system(tab_system, widgets, ui_state);
}

void update_ui_content(UIWidgets &widgets, std::shared_ptr<UIState> ui_state,
                       UiFrameState &frame_state, UiServices &services) {
    uint32_t active_tab = lv_tabview_get_tab_active(frame_state.tabview);
    if (services.battery_source) update_top_bar(widgets, *services.battery_source);

    update_home(widgets, ui_state);
    update_system(widgets, ui_state);

    if (active_tab == 1) {
        update_health_rows(widgets, ui_state);
        bool should_refresh_diag = widgets.diag_rebuild_pending;
        if (++frame_state.diag_frame >= DIAG_REBUILD_INTERVAL) {
            frame_state.diag_frame = 0;
            should_refresh_diag = true;
        }
        if (should_refresh_diag) {
            rebuild_diag_sections(widgets, ui_state);
        }
    } else {
        frame_state.diag_frame = DIAG_REBUILD_INTERVAL;
    }

    update_debug(widgets, ui_state, frame_state.image_dsc, frame_state.image_copy,
                 services.debug_overlay_renderer.get());
}
}  // namespace ui_internal
}  // namespace auto_battlebot
