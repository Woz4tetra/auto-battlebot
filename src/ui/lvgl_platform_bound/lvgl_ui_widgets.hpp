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

/* Width in pixels of the stick position bars flush against the autonomy tile edges. Wide enough
 * to read at a glance from across the pit, narrow enough to leave the label room. */
constexpr int STICK_BAR_THICKNESS = 14;

struct UIWidgets;

struct OpponentTileData {
    UIWidgets *widgets = nullptr;
    int count = 0;
};

struct SystemActionTileData {
    UIWidgets *widgets = nullptr;
    SystemAction action = SystemAction::REBOOT_HOST;
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
    std::time_t last_clock_second = -1;

    lv_obj_t *status_tile = nullptr; /* whole tile colored by status */
    lv_obj_t *status_label = nullptr;
    lv_obj_t *status_detail = nullptr;

    lv_obj_t *profile_readout = nullptr; /* active config profile, top bar (set in update_home) */
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
    /* Stick positions the transmitter reports, drawn flush against the autonomy tile edges. */
    lv_obj_t *stick_bar_left = nullptr;
    lv_obj_t *stick_bar_right = nullptr;

    lv_obj_t *reinit_tile = nullptr;
    lv_obj_t *reinit_status = nullptr;

    lv_obj_t *recording_tile = nullptr;
    lv_obj_t *recording_label = nullptr;
    lv_obj_t *recording_detail = nullptr;

    lv_obj_t *reboot_tile = nullptr;
    lv_obj_t *poweroff_tile = nullptr;
    lv_obj_t *confirm_overlay = nullptr;
    lv_obj_t *confirm_message = nullptr;
    std::optional<SystemAction> pending_confirm_action;

    lv_obj_t *profile_dropdown = nullptr;
    std::vector<std::string> profile_options; /* dropdown index -> profile id */
    lv_obj_t *notice_overlay = nullptr;
    lv_obj_t *notice_message = nullptr;
    std::string last_profile_notice; /* last notice shown, to detect fresh ones */

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
    bool diag_user_scrolling = false;
    bool diag_rebuild_pending = false;
    std::vector<std::string> diag_section_order;
    std::map<std::string,
             std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        diag_section_cache;
    std::array<OpponentTileData, 3> opp_tile_data = {};
    std::array<SystemActionTileData, 2> system_action_tile_data = {};
    CameraTouchData camera_touch_data = {};
};

}  // namespace auto_battlebot::ui_internal
