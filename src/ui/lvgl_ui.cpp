#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <lvgl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <vector>

#include "enums/label.hpp"
#include "label_utils.hpp"
#include "transform_utils.hpp"
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {
namespace {
constexpr int TAB_BAR_HEIGHT = 56;
constexpr int TILE_RADIUS = 12;
constexpr int TILE_PAD = 16;
constexpr int DIAG_REBUILD_INTERVAL = 30;
/** Sections not updated in this many seconds are shown gray (stale). */
constexpr double DIAG_STALE_SEC = 2.0;
constexpr int CAMERA_PANEL_WIDTH_PCT = 66;

static int selected_opponent_count = 0;

/** Persistent cache: section key -> (snapshot, last update time). Never remove
 * entries so scroll/layout stay stable when sections disappear. */
static std::vector<std::string> diag_section_order;
static std::map<std::string,
                std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
    diag_section_cache;

struct OpponentTileData {
    std::shared_ptr<UIState> ui_state;
    int count;
};
static OpponentTileData opp_tile_data[3];

struct CameraTouchData {
    std::shared_ptr<UIState> ui_state;
    struct UIWidgets *widgets = nullptr;
};
static CameraTouchData camera_touch_data;

struct HealthRow {
    lv_obj_t *led = nullptr;
    lv_obj_t *label = nullptr;
};

struct UIWidgets {
    lv_obj_t *tabview = nullptr;

    lv_obj_t *status_tile = nullptr; /* whole tile colored by status */
    lv_obj_t *status_label = nullptr;
    lv_obj_t *status_detail = nullptr;
    lv_obj_t *opp_tiles[3] = {};
    lv_obj_t *opp_label = nullptr;
    lv_obj_t *manual_target_label = nullptr;
    lv_obj_t *camera_frame = nullptr;
    lv_obj_t *camera_touch = nullptr;
    bool manual_target_active = false;
    int camera_src_width = 0;
    int camera_src_height = 0;

    lv_obj_t *autonomy_tile = nullptr;
    lv_obj_t *autonomy_label = nullptr;

    lv_obj_t *reinit_tile = nullptr;
    lv_obj_t *reinit_status = nullptr;

    HealthRow health[8];
    int health_count = 0;
    lv_obj_t *sections_cont = nullptr;

    lv_obj_t *debug_img = nullptr;
    lv_obj_t *debug_kp_cont = nullptr;
    std::vector<lv_obj_t *> kp_dots;

    /** Rolling buffer for loop_rate_hz; size capped by rate_avg_window from
     * UIState. */
    std::deque<double> rate_history;
    /** When rate first went below threshold; empty when above. Used for
     * sustained-fail (anti-strobe). */
    std::optional<std::chrono::steady_clock::time_point> rate_below_since;
    bool reinit_pulsing = false;
};

/** Push current rate into rolling history (call once per frame). Window size
 * from UIState. */
void update_rate_history(UIWidgets &w, std::shared_ptr<UIState> us, double current_hz) {
    if (!us) return;
    int window = us->get_rate_avg_window();
    if (window <= 0) window = 1;
    w.rate_history.push_back(current_hz);
    while (static_cast<int>(w.rate_history.size()) > window) w.rate_history.pop_front();
}

/** Return rolling average Hz from current history (no push). */
double get_rate_avg(const UIWidgets &w, double fallback_hz) {
    if (w.rate_history.empty()) return fallback_hz;
    double sum = 0.0;
    for (double v : w.rate_history) sum += v;
    return sum / static_cast<double>(w.rate_history.size());
}

/** Only false when rate has been below threshold for at least
 * rate_fail_duration_sec (prevents strobing). */
bool compute_loop_met_sustained(UIWidgets &w, std::shared_ptr<UIState> us, double avg_hz) {
    if (!us) return true;
    double max_hz = us->get_max_loop_rate();
    double thresh = us->get_rate_fail_threshold();
    double threshold_hz = max_hz * thresh;
    double duration_sec = us->get_rate_fail_duration_sec();
    auto now = std::chrono::steady_clock::now();

    if (avg_hz >= threshold_hz) {
        w.rate_below_since.reset();
        return true;
    }
    if (!w.rate_below_since.has_value()) w.rate_below_since = now;
    auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - *w.rate_below_since)
            .count();
    return elapsed < duration_sec;
}

void derive_robot_counts(const RobotDescriptionsStamped &robots, bool &our_seen, int &opp_count) {
    our_seen = false;
    opp_count = 0;
    for (const auto &r : robots.descriptions) {
        if (r.label == Label::MR_STABS_MK1 || r.label == Label::MR_STABS_MK2 ||
            r.label == Label::MRS_BUFF_MK1 || r.label == Label::MRS_BUFF_MK2)
            our_seen = true;
        if (r.label == Label::OPPONENT || r.label == Label::HOUSE_BOT) opp_count++;
    }
}

void reinit_cb(lv_event_t *e) {
    auto *us = static_cast<UIState *>(lv_event_get_user_data(e));
    if (us) us->reinit_requested.store(true);
}

void opp_cb(lv_event_t *e) {
    auto *d = static_cast<OpponentTileData *>(lv_event_get_user_data(e));
    if (d && d->ui_state) {
        d->ui_state->opponent_count_requested.store(d->count);
        selected_opponent_count = d->count;
    }
}

void autonomy_cb(lv_event_t *e) {
    auto *us = static_cast<UIState *>(lv_event_get_user_data(e));
    if (!us) return;
    SystemStatus st;
    us->get_system_status(st);
    us->autonomy_toggle_requested.store(st.autonomy_enabled ? -1 : 1);
}

void style_transparent(lv_obj_t *obj) {
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
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

cv::Scalar to_cv_bgr(Label label) {
    ColorRGBf color = get_color_for_label(label);
    return cv::Scalar(color.b * 255.0f, color.g * 255.0f, color.r * 255.0f);
}

bool project_field_point_to_pixel(const FieldDescription &field, const CameraInfo &camera_info,
                                  double x, double y, double z, cv::Point &out) {
    if (field.tf_camera_from_fieldcenter.tf.rows() < 3 ||
        field.tf_camera_from_fieldcenter.tf.cols() < 4) {
        return false;
    }
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) {
        return false;
    }
    if (camera_info.width <= 0 || camera_info.height <= 0) return false;

    const auto &tf = field.tf_camera_from_fieldcenter.tf;
    const double cx = tf(0, 0) * x + tf(0, 1) * y + tf(0, 2) * z + tf(0, 3);
    const double cy = tf(1, 0) * x + tf(1, 1) * y + tf(1, 2) * z + tf(1, 3);
    const double cz = tf(2, 0) * x + tf(2, 1) * y + tf(2, 2) * z + tf(2, 3);
    if (cz <= 1e-6) return false;

    const double fx = camera_info.intrinsics.at<double>(0, 0);
    const double fy = camera_info.intrinsics.at<double>(1, 1);
    const double px0 = camera_info.intrinsics.at<double>(0, 2);
    const double py0 = camera_info.intrinsics.at<double>(1, 2);

    const double u = fx * (cx / cz) + px0;
    const double v = fy * (cy / cz) + py0;
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    out = cv::Point(static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v)));
    return true;
}

void draw_robot_pose_arrows(cv::Mat &image, const RobotDescriptionsStamped &robots,
                            const FieldDescription &field, const CameraInfo &camera_info) {
    for (const auto &robot : robots.descriptions) {
        Pose2D pose2d = pose_to_pose2d(robot.pose);
        const double arrow_len_m = std::max(robot.size.x * 1.5, 0.1);
        const double z = robot.pose.position.z + 0.01;

        cv::Point start_px;
        cv::Point end_px;
        if (!project_field_point_to_pixel(field, camera_info, pose2d.x, pose2d.y, z, start_px))
            continue;
        if (!project_field_point_to_pixel(
                field, camera_info, pose2d.x + arrow_len_m * std::cos(pose2d.yaw),
                pose2d.y + arrow_len_m * std::sin(pose2d.yaw), z, end_px)) {
            continue;
        }
        cv::arrowedLine(image, start_px, end_px, to_cv_bgr(robot.label), 2, cv::LINE_AA, 0, 0.25);
    }
}

void draw_field_border(cv::Mat &image, const FieldDescription &field,
                       const CameraInfo &camera_info) {
    const double hx = field.size.size.x / 2.0;
    const double hy = field.size.size.y / 2.0;
    const std::array<cv::Point3d, 4> corners = {{
        {-hx, -hy, 0.0},
        {hx, -hy, 0.0},
        {hx, hy, 0.0},
        {-hx, hy, 0.0},
    }};

    std::array<cv::Point, 4> projected;
    for (size_t i = 0; i < corners.size(); ++i) {
        if (!project_field_point_to_pixel(field, camera_info, corners[i].x, corners[i].y,
                                          corners[i].z, projected[i])) {
            return;
        }
    }

    for (size_t i = 0; i < projected.size(); ++i) {
        const cv::Point &a = projected[i];
        const cv::Point &b = projected[(i + 1) % projected.size()];
        cv::line(image, a, b, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
}

void draw_target_path_overlay(cv::Mat &image, const std::optional<NavigationPathSegment> &path,
                              const FieldDescription &field, const CameraInfo &camera_info) {
    if (!path.has_value()) return;

    cv::Point our_px;
    cv::Point target_px;
    if (project_field_point_to_pixel(field, camera_info, path->our_x, path->our_y, 0.01, our_px) &&
        project_field_point_to_pixel(field, camera_info, path->target_x, path->target_y, 0.01,
                                     target_px)) {
        cv::line(image, our_px, target_px, cv::Scalar(0, 153, 255), 2, cv::LINE_AA);
    }

    if (project_field_point_to_pixel(field, camera_info, path->target_x, path->target_y, 0.01,
                                     target_px)) {
        constexpr int cross_half = 9;
        cv::line(image, cv::Point(target_px.x - cross_half, target_px.y - cross_half),
                 cv::Point(target_px.x + cross_half, target_px.y + cross_half),
                 cv::Scalar(51, 51, 255), 2, cv::LINE_AA);
        cv::line(image, cv::Point(target_px.x - cross_half, target_px.y + cross_half),
                 cv::Point(target_px.x + cross_half, target_px.y - cross_half),
                 cv::Scalar(51, 51, 255), 2, cv::LINE_AA);
    }
}

void set_manual_target_ui_state(UIWidgets &w, bool active) {
    w.manual_target_active = active;
    if (w.camera_frame) {
        lv_obj_set_style_border_width(w.camera_frame, active ? 4 : 2, 0);
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
                                  int img_x, int img_y, Pose2D &out) {
    if (field.tf_camera_from_fieldcenter.tf.rows() < 4 ||
        field.tf_camera_from_fieldcenter.tf.cols() < 4) {
        return false;
    }
    if (camera_info.intrinsics.rows != 3 || camera_info.intrinsics.cols != 3) return false;

    const double fx = camera_info.intrinsics.at<double>(0, 0);
    const double fy = camera_info.intrinsics.at<double>(1, 1);
    const double cx = camera_info.intrinsics.at<double>(0, 2);
    const double cy = camera_info.intrinsics.at<double>(1, 2);
    if (std::abs(fx) < 1e-6 || std::abs(fy) < 1e-6) return false;

    const Eigen::Vector3d ray((static_cast<double>(img_x) - cx) / fx,
                              (static_cast<double>(img_y) - cy) / fy, 1.0);

    const auto &tf_any = field.tf_camera_from_fieldcenter.tf;
    Eigen::Matrix4d tf = tf_any.block<4, 4>(0, 0);
    const Eigen::Vector4d origin_field(0.0, 0.0, 0.0, 1.0);
    const Eigen::Vector4d x_axis_field(1.0, 0.0, 0.0, 1.0);
    const Eigen::Vector4d y_axis_field(0.0, 1.0, 0.0, 1.0);

    const Eigen::Vector3d p0 = (tf * origin_field).head<3>();
    const Eigen::Vector3d px = (tf * x_axis_field).head<3>();
    const Eigen::Vector3d py = (tf * y_axis_field).head<3>();

    Eigen::Vector3d normal = (px - p0).cross(py - p0);
    const double normal_norm = normal.norm();
    if (normal_norm < 1e-6) return false;
    normal /= normal_norm;

    const double den = ray.dot(normal);
    if (std::abs(den) < 1e-6) return false;
    const double t = p0.dot(normal) / den;
    if (t <= 0.0) return false;

    const Eigen::Vector3d camera_pt = ray * t;
    const Eigen::Vector4d camera_h(camera_pt.x(), camera_pt.y(), camera_pt.z(), 1.0);
    const Eigen::Vector4d field_h = tf.inverse() * camera_h;
    if (!std::isfinite(field_h.x()) || !std::isfinite(field_h.y())) return false;

    out.x = field_h.x();
    out.y = field_h.y();
    out.yaw = 0.0;
    return true;
}

void camera_touch_cb(lv_event_t *e) {
    auto *d = static_cast<CameraTouchData *>(lv_event_get_user_data(e));
    if (!d || !d->ui_state || !d->widgets) return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        d->ui_state->set_manual_target(std::nullopt);
        set_manual_target_ui_state(*d->widgets, false);
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;

    int img_x = 0, img_y = 0;
    if (!event_to_image_pixel(*d->widgets, e, img_x, img_y)) return;

    std::optional<FieldDescription> field;
    d->ui_state->get_field_description(field);
    CameraInfo camera_info;
    d->ui_state->get_camera_info(camera_info);
    if (!field.has_value()) return;

    Pose2D target_pose_field;
    if (!project_image_pixel_to_field(*field, camera_info, img_x, img_y, target_pose_field)) return;
    TargetSelection target{target_pose_field, Label::EMPTY};
    d->ui_state->set_manual_target(target);
    set_manual_target_ui_state(*d->widgets, true);
}

/* ------------------------------------------------------------------ */
/*  Home tab                                                          */
/* ------------------------------------------------------------------ */

void build_home(lv_obj_t *tab, UIWidgets &w, std::shared_ptr<UIState> ui_state) {
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(tab, 8, 0);
    lv_obj_set_style_pad_gap(tab, 8, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *controls = lv_obj_create(tab);
    lv_obj_set_width(controls, LV_PCT(100 - CAMERA_PANEL_WIDTH_PCT));
    lv_obj_set_height(controls, LV_PCT(100));
    lv_obj_set_flex_flow(controls, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(controls, 0, 0);
    lv_obj_set_style_pad_gap(controls, 8, 0);
    style_transparent(controls);

    lv_obj_t *camera_col = lv_obj_create(tab);
    lv_obj_set_width(camera_col, LV_PCT(CAMERA_PANEL_WIDTH_PCT));
    lv_obj_set_height(camera_col, LV_PCT(100));
    lv_obj_set_flex_flow(camera_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(camera_col, 0, 0);
    lv_obj_clear_flag(camera_col, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(camera_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(camera_col, 0, 0);

    lv_obj_t *top = lv_obj_create(controls);
    lv_obj_set_width(top, LV_PCT(100));
    lv_obj_set_flex_grow(top, 2);
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
    lv_obj_add_event_cb(at, autonomy_cb, LV_EVENT_CLICKED, ui_state.get());

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
    lv_obj_add_event_cb(rt, reinit_cb, LV_EVENT_CLICKED, ui_state.get());

    lv_obj_t *ri = lv_label_create(rt);
    lv_label_set_text(ri, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_font(ri, &lv_font_montserrat_32, 0);

    lv_obj_t *rl = lv_label_create(rt);
    lv_label_set_text(rl, "Reinitialize\nField");
    lv_obj_set_style_text_font(rl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(rl, LV_TEXT_ALIGN_CENTER, 0);

    w.reinit_status = lv_label_create(rt);
    lv_label_set_text(w.reinit_status, "");
    lv_obj_set_style_text_font(w.reinit_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(w.reinit_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(w.reinit_status, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t *opp_section = lv_obj_create(controls);
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
        opp_tile_data[i] = {ui_state, i + 1};

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
        lv_obj_add_event_cb(tile, opp_cb, LV_EVENT_CLICKED, &opp_tile_data[i]);

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

    camera_touch_data.ui_state = ui_state;
    camera_touch_data.widgets = &w;
    lv_obj_add_event_cb(w.camera_frame, camera_touch_cb, LV_EVENT_PRESSED, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_frame, camera_touch_cb, LV_EVENT_PRESSING, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_frame, camera_touch_cb, LV_EVENT_RELEASED, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_frame, camera_touch_cb, LV_EVENT_PRESS_LOST, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_PRESSED, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_PRESSING, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_RELEASED, &camera_touch_data);
    lv_obj_add_event_cb(w.camera_touch, camera_touch_cb, LV_EVENT_PRESS_LOST, &camera_touch_data);

    w.debug_img = lv_image_create(w.camera_touch);
    lv_obj_align(w.debug_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(w.debug_img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(w.debug_img, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_event_cb(w.debug_img, camera_touch_cb, LV_EVENT_PRESSED, &camera_touch_data);
    lv_obj_add_event_cb(w.debug_img, camera_touch_cb, LV_EVENT_PRESSING, &camera_touch_data);
    lv_obj_add_event_cb(w.debug_img, camera_touch_cb, LV_EVENT_RELEASED, &camera_touch_data);
    lv_obj_add_event_cb(w.debug_img, camera_touch_cb, LV_EVENT_PRESS_LOST, &camera_touch_data);

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
    lv_obj_add_event_cb(w.debug_kp_cont, camera_touch_cb, LV_EVENT_PRESSED, &camera_touch_data);
    lv_obj_add_event_cb(w.debug_kp_cont, camera_touch_cb, LV_EVENT_PRESSING, &camera_touch_data);
    lv_obj_add_event_cb(w.debug_kp_cont, camera_touch_cb, LV_EVENT_RELEASED, &camera_touch_data);
    lv_obj_add_event_cb(w.debug_kp_cont, camera_touch_cb, LV_EVENT_PRESS_LOST, &camera_touch_data);

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
    derive_robot_counts(robots, our_seen, opp);
    if (st.selected_opponent_count >= 1 && st.selected_opponent_count <= 3) {
        selected_opponent_count = st.selected_opponent_count;
    }

    update_rate_history(w, us, st.loop_rate_hz);
    double avg_hz = get_rate_avg(w, st.loop_rate_hz);
    bool loop_met = compute_loop_met_sustained(w, us, avg_hz);
    bool hardware_ok = st.camera_ok && st.transmitter_connected;
    bool needs_reinit = !st.initialized;
    bool has_fault = !hardware_ok || !loop_met;
    bool tracking = our_seen && opp >= 1;

    /* Green = fully ready + tracking; yellow = action/info; red = hardware or
     * runtime fault */
    if (w.status_tile) {
        lv_color_t tile_color;
        if (has_fault)
            tile_color = lv_color_hex(0xFF1744); /* red */
        else if (!needs_reinit && tracking)
            tile_color = lv_color_hex(0x00C853); /* green */
        else
            tile_color = lv_color_hex(0xFFC107); /* yellow */
        lv_obj_set_style_bg_color(w.status_tile, tile_color, 0);
    }
    if (w.status_label) {
        if (!hardware_ok) {
            lv_label_set_text(w.status_label, "Hardware Disconnected");
        } else if (!loop_met) {
            lv_label_set_text(w.status_label, "Loop Rate Low");
        } else if (needs_reinit) {
            lv_label_set_text(w.status_label, "Press Reinitialize");
        } else {
            lv_label_set_text(w.status_label, "System Ready");
        }
        /* Dark text on yellow/green for readability */
        lv_obj_set_style_text_color(w.status_label,
                                    has_fault ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x212121), 0);
    }

    if (w.status_detail) {
        lv_obj_set_style_text_color(w.status_detail,
                                    has_fault ? lv_color_hex(0xEEEEEE) : lv_color_hex(0x424242), 0);
        char buf[256];
        if (!hardware_ok || !loop_met) {
            std::string details;
            if (!st.camera_ok) details += "Camera disconnected  ";
            if (!st.transmitter_connected) details += "Transmitter disconnected  ";
            if (!loop_met) {
                char rate_buf[64];
                snprintf(rate_buf, sizeof(rate_buf), "Loop slow (%.1f Hz)  ", avg_hz);
                details += rate_buf;
            }
            if (needs_reinit && hardware_ok) details += "Press Reinitialize Field  ";
            snprintf(buf, sizeof(buf), "%s", details.c_str());
        } else if (needs_reinit) {
            snprintf(buf, sizeof(buf), "Press Reinitialize Field to start");
        } else {
            snprintf(buf, sizeof(buf), "%.1f Hz  |  %s  |  %d opponent%s seen", avg_hz,
                     our_seen ? "our robot in view" : "our robot not seen", opp,
                     opp != 1 ? "s" : "");
        }
        lv_label_set_text(w.status_detail, buf);
    }

    if (w.reinit_tile) {
        lv_color_t reinit_color = lv_color_hex(0x00C853);
        if (!st.initialized) {
            reinit_color = hardware_ok ? lv_color_hex(0xFFC107) : lv_color_hex(0xFF1744);
        }
        lv_obj_set_style_bg_color(w.reinit_tile, reinit_color, 0);
    }
    if (w.reinit_status) {
        if (st.initialized) {
            lv_label_set_text(w.reinit_status, "field initialized");
        } else if (hardware_ok) {
            lv_label_set_text(w.reinit_status, "press to initialize field");
        } else {
            lv_label_set_text(w.reinit_status, "reconnect camera/transmitter");
        }
    }

    if (hardware_ok && !st.initialized) {
        start_reinit_pulse(w);
    } else {
        stop_reinit_pulse(w);
    }

    if (w.autonomy_tile) {
        lv_obj_set_style_bg_color(
            w.autonomy_tile, st.autonomy_enabled ? lv_color_hex(0x00C853) : lv_color_hex(0xFF1744),
            0);
    }
    if (w.autonomy_label) {
        lv_label_set_text(w.autonomy_label, st.autonomy_enabled ? "Autonomy\nON" : "Autonomy\nOFF");
        lv_obj_set_style_text_color(
            w.autonomy_label, st.autonomy_enabled ? lv_color_hex(0x212121) : lv_color_hex(0xFFFFFF),
            0);
    }

    for (int i = 0; i < 3; i++) {
        if (!w.opp_tiles[i]) continue;
        if (selected_opponent_count == i + 1)
            lv_obj_add_state(w.opp_tiles[i], LV_STATE_CHECKED);
        else
            lv_obj_remove_state(w.opp_tiles[i], LV_STATE_CHECKED);
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
    derive_robot_counts(robots, our_seen, opp);

    char buf[128];

    snprintf(buf, sizeof(buf), "Camera: %s", st.camera_ok ? "Connected" : "Disconnected");
    set_health(w.health[0], st.camera_ok, buf);

    snprintf(buf, sizeof(buf), "Transmitter: %s",
             st.transmitter_connected ? "Connected" : "Disconnected");
    set_health(w.health[1], st.transmitter_connected, buf);

    if (st.initialized) {
        snprintf(buf, sizeof(buf), "Field: Initialized");
    } else if (st.camera_ok && st.transmitter_connected) {
        snprintf(buf, sizeof(buf), "Field: Press Reinitialize");
    } else {
        snprintf(buf, sizeof(buf), "Field: Waiting for camera/transmitter");
    }
    set_health(w.health[2], st.initialized || (st.camera_ok && st.transmitter_connected), buf);

    snprintf(buf, sizeof(buf), "Our Robot Seen: %s", our_seen ? "Yes" : "No");
    set_health(w.health[3], our_seen, buf);

    snprintf(buf, sizeof(buf), "Opponents Seen: %d", opp);
    set_health(w.health[4], opp > 0, buf);

    double avg_hz = get_rate_avg(w, st.loop_rate_hz);
    bool loop_met = compute_loop_met_sustained(w, us, avg_hz);
    snprintf(buf, sizeof(buf), "Loop Rate: %.1f Hz %s", avg_hz, loop_met ? "(met)" : "(NOT MET)");
    set_health(w.health[5], loop_met, buf);

    if (st.jetson_temperature_c > 0.0) {
        snprintf(buf, sizeof(buf), "Jetson Temp: %.1f C", st.jetson_temperature_c);
        set_health(w.health[6], st.jetson_temperature_c < 80.0, buf);
    }

    if (!st.jetson_compute_mode.empty()) {
        snprintf(buf, sizeof(buf), "Compute Mode: %s", st.jetson_compute_mode.c_str());
        set_health(w.health[7], true, buf);
    }
}

static std::string diag_section_key(const DiagnosticStatusSnapshot &snap) {
    std::string key = snap.name;
    if (!snap.subsection.empty()) key += "\x01" + snap.subsection;
    return key;
}

void rebuild_diag_sections(UIWidgets &w, std::shared_ptr<UIState> us) {
    if (!us || !w.sections_cont) return;

    const auto now = std::chrono::steady_clock::now();

    /* Merge new snapshots into cache; keep old entries, never remove */
    std::vector<DiagnosticStatusSnapshot> snaps;
    us->get_diagnostic_snapshots(snaps);
    for (const auto &snap : snaps) {
        std::string key = diag_section_key(snap);
        auto it = diag_section_cache.find(key);
        if (it == diag_section_cache.end()) {
            diag_section_order.push_back(key);
            diag_section_cache[key] = {snap, now};
        } else {
            it->second.first = snap;
            it->second.second = now;
        }
    }

    /* Preserve scroll position when rebuilding content */
    lv_obj_t *diag_tab = lv_obj_get_parent(w.sections_cont);
    const int32_t scroll_y = (diag_tab != nullptr) ? lv_obj_get_scroll_top(diag_tab) : 0;

    lv_obj_clean(w.sections_cont);

    for (const std::string &key : diag_section_order) {
        auto it = diag_section_cache.find(key);
        if (it == diag_section_cache.end()) continue;
        const DiagnosticStatusSnapshot &snap = it->second.first;
        const std::chrono::steady_clock::time_point &updated = it->second.second;
        const double age_sec = std::chrono::duration<double>(now - updated).count();
        const bool stale = (age_sec > DIAG_STALE_SEC);

        std::string title = snap.name;
        if (!snap.subsection.empty()) title += ": " + snap.subsection;

        /* Section header bar: green when fresh, gray when stale */
        lv_obj_t *hdr = lv_obj_create(w.sections_cont);
        lv_obj_set_size(hdr, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(hdr, stale ? lv_color_hex(0x757575) : lv_color_hex(0x4CAF50), 0);
        lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(hdr, 4, 0);
        lv_obj_set_style_radius(hdr, 4, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *hlbl = lv_label_create(hdr);
        lv_label_set_text(hlbl, title.c_str());
        lv_obj_set_style_text_font(hlbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(hlbl, lv_color_hex(0xFFFFFF), 0);

        size_t rows = snap.values.size() + (snap.message.empty() ? 0 : 1);
        if (rows == 0) continue;

        lv_obj_t *table = lv_table_create(w.sections_cont);
        lv_table_set_column_count(table, 2);
        lv_table_set_row_count(table, static_cast<uint32_t>(rows));
        lv_table_set_column_width(table, 0, 320);
        lv_table_set_column_width(table, 1, 480);
        lv_obj_set_width(table, LV_PCT(100));
        lv_obj_set_style_text_font(table, &lv_font_montserrat_20, LV_PART_ITEMS);
        lv_obj_set_style_pad_all(table, 2, LV_PART_ITEMS);
        if (stale) lv_obj_set_style_text_color(table, lv_color_hex(0x9E9E9E), LV_PART_ITEMS);

        uint32_t row = 0;
        if (!snap.message.empty()) {
            lv_table_set_cell_value(table, row, 0, "message");
            lv_table_set_cell_value(table, row, 1, snap.message.c_str());
            row++;
        }
        for (const auto &[k, v] : snap.values) {
            lv_table_set_cell_value(table, row, 0, k.c_str());
            lv_table_set_cell_value(table, row, 1, v.c_str());
            row++;
        }
    }

    if (diag_tab != nullptr) lv_obj_scroll_to_y(diag_tab, scroll_y, LV_ANIM_OFF);
}

void update_debug(UIWidgets &w, std::shared_ptr<UIState> us, lv_image_dsc_t &img_dsc,
                  std::vector<uint8_t> &img_copy) {
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
            img_copy = data;
            if (field_description.has_value()) {
                if (camera_info.width <= 0 || camera_info.height <= 0) {
                    camera_info.width = dw;
                    camera_info.height = dh;
                }
                cv::Mat image(dh, dw, dc == 4 ? CV_8UC4 : CV_8UC3, img_copy.data());
                draw_field_border(image, *field_description, camera_info);
                draw_robot_pose_arrows(image, robots, *field_description, camera_info);
                draw_target_path_overlay(image, navigation_path, *field_description, camera_info);
            }
            img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
            img_dsc.header.cf = (dc == 4) ? LV_COLOR_FORMAT_XRGB8888 : LV_COLOR_FORMAT_RGB888;
            img_dsc.header.flags = 0;
            img_dsc.header.w = static_cast<uint32_t>(dw);
            img_dsc.header.h = static_cast<uint32_t>(dh);
            img_dsc.header.stride = dw * (dc == 4 ? 4 : 3);
            img_dsc.data_size = static_cast<uint32_t>(img_copy.size());
            img_dsc.data = img_copy.data();
            w.camera_src_width = dw;
            w.camera_src_height = dh;
            lv_image_set_src(w.debug_img, &img_dsc);

            /* Scale image to fit container without cropping; preserve aspect ratio (shrink only) */
            lv_obj_t *cont = lv_obj_get_parent(w.debug_img);
            const int32_t cont_w = cont ? lv_obj_get_content_width(cont) : dw;
            const int32_t cont_h = cont ? lv_obj_get_content_height(cont) : dh;
            const double scale = std::min(
                1.0, std::min(static_cast<double>(cont_w) / dw, static_cast<double>(cont_h) / dh));
            const int32_t sw = static_cast<int32_t>(std::round(dw * scale));
            const int32_t sh = static_cast<int32_t>(std::round(dh * scale));
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

void run_ui_thread(std::shared_ptr<UIState> ui_state) {
    if (!ui_state) return;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) return;

    int w = 1280, h = 800;
    ui_state->get_window_size(w, h);
    if (w <= 0 || h <= 0) {
        w = 1280;
        h = 800;
    }

    lv_init();
    lv_display_t *disp = lv_sdl_window_create(static_cast<int32_t>(w), static_cast<int32_t>(h));
    if (!disp) {
        SDL_Quit();
        return;
    }
    lv_sdl_window_set_title(disp, "Auto BattleBot");

    if (ui_state->get_fullscreen()) {
        /* LVGL SDL driver stores driver_data as lv_sdl_window_t with window as first member */
        void *driver_data = lv_display_get_driver_data(disp);
        if (driver_data) {
            SDL_Window *sdl_win = *static_cast<SDL_Window **>(driver_data);
            if (sdl_win) SDL_SetWindowFullscreen(sdl_win, SDL_WINDOW_FULLSCREEN_DESKTOP);
        }
    }

    lv_indev_t *mouse = lv_sdl_mouse_create();
    lv_indev_t *mousewheel = lv_sdl_mousewheel_create();
    lv_indev_t *keyboard = lv_sdl_keyboard_create();
    (void)mouse;
    (void)mousewheel;
    (void)keyboard;

    lv_obj_t *screen = lv_screen_active();
    if (!screen) {
        SDL_Quit();
        return;
    }

    lv_obj_t *tabview = lv_tabview_create(screen);
    lv_obj_set_size(tabview, LV_PCT(100), LV_PCT(100));
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

    UIWidgets widgets = {};
    widgets.tabview = tabview;
    build_home(tab_home, widgets, ui_state);
    build_diagnostics(tab_diag, widgets);

    lv_image_dsc_t img_dsc = {};
    std::vector<uint8_t> img_copy;
    int diag_frame = 0;

    bool running = true;
    const int delay_ms = 5;
    while (running) {
        if (ui_state->quit_requested.load()) break;

        SDL_Event event;
        if (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT ||
                (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE)) {
                running = false;
            } else {
                SDL_PushEvent(&event);
            }
        }

        ui_state->quit_requested.store(!running);
        if (!running) break;

        uint32_t active_tab = lv_tabview_get_tab_active(tabview);

        update_home(widgets, ui_state);

        if (active_tab == 1) {
            update_health_rows(widgets, ui_state);
            if (++diag_frame >= DIAG_REBUILD_INTERVAL) {
                diag_frame = 0;
                rebuild_diag_sections(widgets, ui_state);
            }
        } else {
            diag_frame = DIAG_REBUILD_INTERVAL;
        }

        update_debug(widgets, ui_state, img_dsc, img_copy);

        lv_tick_inc(delay_ms);
        lv_timer_handler();
        SDL_Delay(delay_ms);
    }

    lv_sdl_quit();
    SDL_Quit();
}
}  // namespace auto_battlebot
