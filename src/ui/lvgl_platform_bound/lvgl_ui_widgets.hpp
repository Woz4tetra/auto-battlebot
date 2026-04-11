#pragma once

#include <lvgl.h>

#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lvgl_ui_controller.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot::ui_internal {

constexpr int TOP_BAR_HEIGHT = 40;
constexpr int TAB_BAR_HEIGHT = 56;
constexpr int TILE_RADIUS = 12;
constexpr int TILE_PAD = 16;
constexpr int DIAG_REBUILD_INTERVAL = 30;
constexpr double DIAG_STALE_SEC = 2.0;
constexpr int CAMERA_PANEL_WIDTH_PCT = 66;

constexpr int BATTERY_ICON_CANVAS_WIDTH = 60;
constexpr int BATTERY_ICON_CANVAS_HEIGHT = 24;
constexpr int BATTERY_ICON_SHELL_X = 3;
constexpr int BATTERY_ICON_SHELL_Y = 2;
constexpr int BATTERY_ICON_SHELL_WIDTH = 44;
constexpr int BATTERY_ICON_SHELL_HEIGHT = 20;
constexpr int BATTERY_ICON_FILL_MAX_WIDTH = 34;
constexpr int BATTERY_ICON_FILL_HEIGHT = 12;
constexpr int BATTERY_ICON_CAP_WIDTH = 6;
constexpr int BATTERY_ICON_CAP_HEIGHT = 10;

struct UIWidgets;

struct OpponentTileData {
    UIWidgets *widgets = nullptr;
    int count = 0;
};

struct SystemActionTileData {
    UIWidgets *widgets = nullptr;
    UISystemAction action = UISystemAction::NONE;
};

struct CameraTouchData {
    UIWidgets *widgets = nullptr;
};

struct HealthRow {
    lv_obj_t *led = nullptr;
    lv_obj_t *label = nullptr;
};

struct UIWidgets {
    lv_obj_t *tabview = nullptr;
    std::shared_ptr<UIState> ui_state;
    std::shared_ptr<UiController> controller;
    lv_obj_t *top_bar = nullptr;
    lv_obj_t *clock_label = nullptr;
    lv_obj_t *battery_percent_label = nullptr;
    lv_obj_t *battery_icon_canvas = nullptr;
    std::array<uint8_t, LV_CANVAS_BUF_SIZE(BATTERY_ICON_CANVAS_WIDTH, BATTERY_ICON_CANVAS_HEIGHT,
                                           LV_COLOR_FORMAT_GET_BPP(LV_COLOR_FORMAT_RGB565),
                                           LV_DRAW_BUF_STRIDE_ALIGN)>
        battery_icon_canvas_buf = {};
    std::time_t last_clock_second = -1;
    std::chrono::steady_clock::time_point last_battery_update =
        std::chrono::steady_clock::time_point::min();
    double dummy_battery_percent = 75.0;
    bool dummy_battery_descending = false;
    std::string battery_source = "waveshare ups";

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

    lv_obj_t *recording_tile = nullptr;
    lv_obj_t *recording_label = nullptr;
    lv_obj_t *recording_detail = nullptr;

    lv_obj_t *quit_tile = nullptr;
    lv_obj_t *reboot_tile = nullptr;
    lv_obj_t *poweroff_tile = nullptr;
    lv_obj_t *confirm_overlay = nullptr;
    lv_obj_t *confirm_message = nullptr;
    UISystemAction pending_confirm_action = UISystemAction::NONE;

    HealthRow health[8];
    int health_count = 0;
    lv_obj_t *sections_cont = nullptr;

    lv_obj_t *debug_img = nullptr;
    lv_obj_t *debug_kp_cont = nullptr;
    std::vector<lv_obj_t *> kp_dots;

    std::deque<double> rate_history;
    std::optional<std::chrono::steady_clock::time_point> rate_below_since;
    bool reinit_pulsing = false;

    int selected_opponent_count = 0;
    std::vector<std::string> diag_section_order;
    std::map<std::string,
             std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        diag_section_cache;
    std::array<OpponentTileData, 3> opp_tile_data = {};
    std::array<SystemActionTileData, 3> system_action_tile_data = {};
    CameraTouchData camera_touch_data = {};
};

}  // namespace auto_battlebot::ui_internal
