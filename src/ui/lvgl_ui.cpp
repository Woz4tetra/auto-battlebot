#define SDL_MAIN_HANDLED
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"
#include "enums/label.hpp"
#include <SDL.h>
#include <lvgl.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace auto_battlebot
{
    namespace
    {
        constexpr int TAB_BAR_HEIGHT = 56;
        constexpr int TILE_RADIUS = 12;
        constexpr int TILE_PAD = 16;
        constexpr int DIAG_REBUILD_INTERVAL = 30;

        static int selected_opponent_count = 0;

        struct OpponentTileData
        {
            std::shared_ptr<UIState> ui_state;
            int count;
        };
        static OpponentTileData opp_tile_data[3];

        struct HealthRow
        {
            lv_obj_t *led = nullptr;
            lv_obj_t *label = nullptr;
        };

        struct UIWidgets
        {
            lv_obj_t *tabview = nullptr;

            lv_obj_t *status_led = nullptr;
            lv_obj_t *status_label = nullptr;
            lv_obj_t *status_detail = nullptr;
            lv_obj_t *opp_tiles[3] = {};

            HealthRow health[8];
            int health_count = 0;
            lv_obj_t *sections_cont = nullptr;

            lv_obj_t *debug_img = nullptr;
            lv_obj_t *debug_kp_cont = nullptr;
            std::vector<lv_obj_t *> kp_dots;
        };

        void derive_robot_counts(const RobotDescriptionsStamped &robots, bool &our_seen, int &opp_count)
        {
            our_seen = false;
            opp_count = 0;
            for (const auto &r : robots.descriptions)
            {
                if (r.label == Label::MR_STABS_MK1 || r.label == Label::MR_STABS_MK2 ||
                    r.label == Label::MRS_BUFF_MK1 || r.label == Label::MRS_BUFF_MK2)
                    our_seen = true;
                if (r.label == Label::OPPONENT || r.label == Label::HOUSE_BOT)
                    opp_count++;
            }
        }

        void reinit_cb(lv_event_t *e)
        {
            auto *us = static_cast<UIState *>(lv_event_get_user_data(e));
            if (us)
                us->reinit_requested.store(true);
        }

        void opp_cb(lv_event_t *e)
        {
            auto *d = static_cast<OpponentTileData *>(lv_event_get_user_data(e));
            if (d && d->ui_state)
            {
                d->ui_state->opponent_count_requested.store(d->count);
                selected_opponent_count = d->count;
            }
        }

        void style_transparent(lv_obj_t *obj)
        {
            lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(obj, 0, 0);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
        }

        HealthRow make_health_row(lv_obj_t *parent, const char *text)
        {
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

        void set_health(HealthRow &hr, bool ok, const char *text)
        {
            if (hr.led)
                lv_led_set_color(hr.led, ok ? lv_color_hex(0x00C853) : lv_color_hex(0xFF1744));
            if (hr.label)
                lv_label_set_text(hr.label, text);
        }

        /* ------------------------------------------------------------------ */
        /*  Home tab                                                          */
        /* ------------------------------------------------------------------ */

        void build_home(lv_obj_t *tab, UIWidgets &w, std::shared_ptr<UIState> ui_state)
        {
            lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(tab, 8, 0);
            lv_obj_set_style_pad_gap(tab, 8, 0);
            lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

            /* --- Top row: status + reinit --- */
            lv_obj_t *top = lv_obj_create(tab);
            lv_obj_set_width(top, LV_PCT(100));
            lv_obj_set_flex_grow(top, 1);
            lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
            lv_obj_set_style_pad_all(top, 0, 0);
            lv_obj_set_style_pad_gap(top, 8, 0);
            style_transparent(top);

            /* Status tile */
            lv_obj_t *st = lv_obj_create(top);
            lv_obj_set_flex_grow(st, 1);
            lv_obj_set_height(st, LV_PCT(100));
            lv_obj_set_style_radius(st, TILE_RADIUS, 0);
            lv_obj_set_style_pad_all(st, TILE_PAD, 0);
            lv_obj_set_flex_flow(st, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(st, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(st, LV_OBJ_FLAG_SCROLLABLE);

            w.status_led = lv_led_create(st);
            lv_obj_set_size(w.status_led, 80, 80);
            lv_led_set_color(w.status_led, lv_color_hex(0x00C853));
            lv_led_on(w.status_led);

            w.status_label = lv_label_create(st);
            lv_label_set_text(w.status_label, "Waiting...");
            lv_obj_set_style_text_font(w.status_label, &lv_font_montserrat_28, 0);

            w.status_detail = lv_label_create(st);
            lv_label_set_text(w.status_detail, "");
            lv_obj_set_style_text_font(w.status_detail, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(w.status_detail, lv_color_hex(0xAAAAAA), 0);

            /* Reinit tile */
            lv_obj_t *rt = lv_obj_create(top);
            lv_obj_set_flex_grow(rt, 1);
            lv_obj_set_height(rt, LV_PCT(100));
            lv_obj_set_style_radius(rt, TILE_RADIUS, 0);
            lv_obj_set_style_pad_all(rt, TILE_PAD, 0);
            lv_obj_set_flex_flow(rt, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(rt, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(rt, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(rt, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_style_bg_color(rt, lv_color_hex(0x1565C0), 0);
            lv_obj_add_event_cb(rt, reinit_cb, LV_EVENT_CLICKED, ui_state.get());

            lv_obj_t *ri = lv_label_create(rt);
            lv_label_set_text(ri, LV_SYMBOL_REFRESH);
            lv_obj_set_style_text_font(ri, &lv_font_montserrat_32, 0);

            lv_obj_t *rl = lv_label_create(rt);
            lv_label_set_text(rl, "Reinitialize\nField");
            lv_obj_set_style_text_font(rl, &lv_font_montserrat_28, 0);
            lv_obj_set_style_text_align(rl, LV_TEXT_ALIGN_CENTER, 0);

            /* --- Bottom row: opponent tiles --- */
            lv_obj_t *bot = lv_obj_create(tab);
            lv_obj_set_width(bot, LV_PCT(100));
            lv_obj_set_flex_grow(bot, 1);
            lv_obj_set_flex_flow(bot, LV_FLEX_FLOW_ROW);
            lv_obj_set_style_pad_all(bot, 0, 0);
            lv_obj_set_style_pad_gap(bot, 8, 0);
            style_transparent(bot);

            for (int i = 0; i < 3; i++)
            {
                opp_tile_data[i] = {ui_state, i + 1};

                lv_obj_t *tile = lv_obj_create(bot);
                lv_obj_set_flex_grow(tile, 1);
                lv_obj_set_height(tile, LV_PCT(100));
                lv_obj_set_style_radius(tile, TILE_RADIUS, 0);
                lv_obj_set_style_pad_all(tile, TILE_PAD, 0);
                lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_add_event_cb(tile, opp_cb, LV_EVENT_CLICKED, &opp_tile_data[i]);

                lv_obj_set_style_border_color(tile, lv_color_hex(0x2979FF), LV_STATE_CHECKED);
                lv_obj_set_style_border_width(tile, 4, LV_STATE_CHECKED);
                lv_obj_set_style_bg_color(tile, lv_color_hex(0x1A237E), LV_STATE_CHECKED);

                lv_obj_t *num = lv_label_create(tile);
                lv_label_set_text_fmt(num, "%d", i + 1);
                lv_obj_set_style_text_font(num, &lv_font_montserrat_32, 0);

                lv_obj_t *desc = lv_label_create(tile);
                lv_label_set_text(desc, (i == 0) ? "Opponent" : "Opponents");
                lv_obj_set_style_text_font(desc, &lv_font_montserrat_20, 0);

                w.opp_tiles[i] = tile;
            }
        }

        /* ------------------------------------------------------------------ */
        /*  Diagnostics tab                                                   */
        /* ------------------------------------------------------------------ */

        void build_diagnostics(lv_obj_t *tab, UIWidgets &w)
        {
            lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(tab, 8, 0);
            lv_obj_set_style_pad_gap(tab, 8, 0);

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
            w.health[2] = make_health_row(hc, "Field Initialized: --");
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
            lv_obj_set_flex_grow(w.sections_cont, 1);
            lv_obj_set_flex_flow(w.sections_cont, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(w.sections_cont, 4, 0);
            lv_obj_set_style_pad_gap(w.sections_cont, 4, 0);
            lv_obj_set_scrollbar_mode(w.sections_cont, LV_SCROLLBAR_MODE_AUTO);
        }

        /* ------------------------------------------------------------------ */
        /*  Debug tab                                                         */
        /* ------------------------------------------------------------------ */

        void build_debug(lv_obj_t *tab, UIWidgets &w)
        {
            lv_obj_t *dc = lv_obj_create(tab);
            lv_obj_set_size(dc, LV_PCT(100), LV_PCT(100));
            lv_obj_set_flex_flow(dc, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(dc, 4, 0);
            lv_obj_clear_flag(dc, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *ic = lv_obj_create(dc);
            lv_obj_set_size(ic, LV_PCT(100), LV_PCT(100));
            lv_obj_set_flex_grow(ic, 1);
            lv_obj_clear_flag(ic, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_pad_all(ic, 0, 0);

            w.debug_img = lv_image_create(ic);
            lv_obj_align(w.debug_img, LV_ALIGN_TOP_LEFT, 0, 0);

            w.debug_kp_cont = lv_obj_create(ic);
            lv_obj_set_size(w.debug_kp_cont, LV_PCT(100), LV_PCT(100));
            lv_obj_align(w.debug_kp_cont, LV_ALIGN_TOP_LEFT, 0, 0);
            lv_obj_clear_flag(w.debug_kp_cont, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_bg_opa(w.debug_kp_cont, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(w.debug_kp_cont, 0, 0);
            lv_obj_set_style_pad_all(w.debug_kp_cont, 0, 0);
            lv_obj_set_style_radius(w.debug_kp_cont, 0, 0);

            w.kp_dots.clear();
        }

        /* ------------------------------------------------------------------ */
        /*  Update functions                                                  */
        /* ------------------------------------------------------------------ */

        void update_home(UIWidgets &w, std::shared_ptr<UIState> us)
        {
            if (!us)
                return;

            SystemStatus st;
            us->get_system_status(st);
            RobotDescriptionsStamped robots;
            us->get_robots(robots);
            bool our_seen = false;
            int opp = 0;
            derive_robot_counts(robots, our_seen, opp);

            bool ok = st.camera_ok && st.transmitter_connected && st.initialized && st.loop_met;

            if (w.status_led)
            {
                lv_led_set_color(w.status_led, ok ? lv_color_hex(0x00C853) : lv_color_hex(0xFF1744));
                lv_led_on(w.status_led);
            }
            if (w.status_label)
                lv_label_set_text(w.status_label, ok ? "System OK" : "System Error");

            if (w.status_detail)
            {
                char buf[256];
                if (ok)
                {
                    snprintf(buf, sizeof(buf), "%.1f Hz  |  %d opponent%s seen",
                             st.loop_rate_hz, opp, opp != 1 ? "s" : "");
                }
                else
                {
                    std::string reasons;
                    if (!st.camera_ok)
                        reasons += "Camera FAIL  ";
                    if (!st.transmitter_connected)
                        reasons += "TX disconnected  ";
                    if (!st.initialized)
                        reasons += "Not initialized  ";
                    if (!st.loop_met)
                        reasons += "Loop slow  ";
                    snprintf(buf, sizeof(buf), "%s", reasons.c_str());
                }
                lv_label_set_text(w.status_detail, buf);
            }

            for (int i = 0; i < 3; i++)
            {
                if (!w.opp_tiles[i])
                    continue;
                if (selected_opponent_count == i + 1)
                    lv_obj_add_state(w.opp_tiles[i], LV_STATE_CHECKED);
                else
                    lv_obj_remove_state(w.opp_tiles[i], LV_STATE_CHECKED);
            }
        }

        void update_health_rows(UIWidgets &w, std::shared_ptr<UIState> us)
        {
            if (!us)
                return;

            SystemStatus st;
            us->get_system_status(st);
            RobotDescriptionsStamped robots;
            us->get_robots(robots);
            bool our_seen = false;
            int opp = 0;
            derive_robot_counts(robots, our_seen, opp);

            char buf[128];

            snprintf(buf, sizeof(buf), "Camera: %s", st.camera_ok ? "OK" : "FAIL");
            set_health(w.health[0], st.camera_ok, buf);

            snprintf(buf, sizeof(buf), "Transmitter: %s", st.transmitter_connected ? "Connected" : "Disconnected");
            set_health(w.health[1], st.transmitter_connected, buf);

            snprintf(buf, sizeof(buf), "Field Initialized: %s", st.initialized ? "Yes" : "No");
            set_health(w.health[2], st.initialized, buf);

            snprintf(buf, sizeof(buf), "Our Robot Seen: %s", our_seen ? "Yes" : "No");
            set_health(w.health[3], our_seen, buf);

            snprintf(buf, sizeof(buf), "Opponents Seen: %d", opp);
            set_health(w.health[4], opp > 0, buf);

            snprintf(buf, sizeof(buf), "Loop Rate: %.1f Hz %s", st.loop_rate_hz, st.loop_met ? "(met)" : "(NOT MET)");
            set_health(w.health[5], st.loop_met, buf);

            if (st.jetson_temperature_c > 0.0)
            {
                snprintf(buf, sizeof(buf), "Jetson Temp: %.1f C", st.jetson_temperature_c);
                set_health(w.health[6], st.jetson_temperature_c < 80.0, buf);
            }

            if (!st.jetson_compute_mode.empty())
            {
                snprintf(buf, sizeof(buf), "Compute Mode: %s", st.jetson_compute_mode.c_str());
                set_health(w.health[7], true, buf);
            }
        }

        void rebuild_diag_sections(UIWidgets &w, std::shared_ptr<UIState> us)
        {
            if (!us || !w.sections_cont)
                return;

            std::vector<DiagnosticStatusSnapshot> snaps;
            us->get_diagnostic_snapshots(snaps);

            lv_obj_clean(w.sections_cont);

            for (const auto &snap : snaps)
            {
                std::string title = snap.name;
                if (!snap.subsection.empty())
                    title += ": " + snap.subsection;

                /* Section header bar */
                lv_obj_t *hdr = lv_obj_create(w.sections_cont);
                lv_obj_set_size(hdr, LV_PCT(100), LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(hdr, lv_color_hex(0x4CAF50), 0);
                lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
                lv_obj_set_style_pad_all(hdr, 6, 0);
                lv_obj_set_style_radius(hdr, 4, 0);
                lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *hlbl = lv_label_create(hdr);
                lv_label_set_text(hlbl, title.c_str());
                lv_obj_set_style_text_font(hlbl, &lv_font_montserrat_20, 0);
                lv_obj_set_style_text_color(hlbl, lv_color_hex(0xFFFFFF), 0);

                /* Key-value table */
                size_t rows = snap.values.size() + (snap.message.empty() ? 0 : 1);
                if (rows == 0)
                    continue;

                lv_obj_t *table = lv_table_create(w.sections_cont);
                lv_table_set_column_count(table, 2);
                lv_table_set_row_count(table, static_cast<uint32_t>(rows));
                lv_table_set_column_width(table, 0, 400);
                lv_table_set_column_width(table, 1, 600);
                lv_obj_set_width(table, LV_PCT(100));

                uint32_t row = 0;
                if (!snap.message.empty())
                {
                    lv_table_set_cell_value(table, row, 0, "message");
                    lv_table_set_cell_value(table, row, 1, snap.message.c_str());
                    row++;
                }
                for (const auto &[k, v] : snap.values)
                {
                    lv_table_set_cell_value(table, row, 0, k.c_str());
                    lv_table_set_cell_value(table, row, 1, v.c_str());
                    row++;
                }
            }
        }

        void update_debug(UIWidgets &w, std::shared_ptr<UIState> us,
                          lv_image_dsc_t &img_dsc, std::vector<uint8_t> &img_copy)
        {
            if (!w.debug_img || !us)
                return;

            int dw = 0, dh = 0, dc = 0;
            std::vector<uint8_t> data;
            us->get_debug_image(dw, dh, dc, data);
            KeypointsStamped kps;
            us->get_keypoints(kps);

            if (dw > 0 && dh > 0 && !data.empty())
            {
                size_t expected = static_cast<size_t>(dw) * dh * (dc == 4 ? 4 : 3);
                if (data.size() >= expected)
                {
                    img_copy = data;
                    img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
                    img_dsc.header.cf = (dc == 4) ? LV_COLOR_FORMAT_XRGB8888 : LV_COLOR_FORMAT_RGB888;
                    img_dsc.header.flags = 0;
                    img_dsc.header.w = static_cast<uint32_t>(dw);
                    img_dsc.header.h = static_cast<uint32_t>(dh);
                    img_dsc.header.stride = dw * (dc == 4 ? 4 : 3);
                    img_dsc.data_size = static_cast<uint32_t>(img_copy.size());
                    img_dsc.data = img_copy.data();
                    lv_image_set_src(w.debug_img, &img_dsc);
                    lv_obj_set_size(w.debug_img, dw, dh);
                    lv_obj_set_size(w.debug_kp_cont, dw, dh);
                }
            }

            uint32_t need = static_cast<uint32_t>(kps.keypoints.size());
            while (w.kp_dots.size() < need)
            {
                lv_obj_t *dot = lv_obj_create(w.debug_kp_cont);
                lv_obj_set_size(dot, 12, 12);
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                lv_obj_set_style_bg_color(dot, lv_color_hex(0x00FF00), 0);
                lv_obj_set_style_border_width(dot, 0, 0);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
                w.kp_dots.push_back(dot);
            }

            for (size_t i = 0; i < w.kp_dots.size(); i++)
            {
                lv_obj_t *dot = w.kp_dots[i];
                if (i < kps.keypoints.size())
                {
                    const auto &kp = kps.keypoints[i];
                    lv_obj_set_pos(dot, static_cast<int32_t>(kp.x) - 6, static_cast<int32_t>(kp.y) - 6);
                    lv_obj_clear_flag(dot, LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }

    } // namespace

    void run_ui_thread(std::shared_ptr<UIState> ui_state)
    {
        if (!ui_state)
            return;

        if (SDL_Init(SDL_INIT_VIDEO) != 0)
            return;

        int w = 1280, h = 800;
        ui_state->get_window_size(w, h);
        if (w <= 0 || h <= 0)
        {
            w = 1280;
            h = 800;
        }

        lv_init();
        lv_display_t *disp = lv_sdl_window_create(static_cast<int32_t>(w), static_cast<int32_t>(h));
        if (!disp)
        {
            SDL_Quit();
            return;
        }
        lv_sdl_window_set_title(disp, "Auto BattleBot");

        if (ui_state->get_fullscreen())
        {
            SDL_Window *sdl_win = SDL_GetWindowFromID(1);
            if (sdl_win)
                SDL_SetWindowFullscreen(sdl_win, SDL_WINDOW_FULLSCREEN_DESKTOP);
        }

        lv_indev_t *mouse = lv_sdl_mouse_create();
        lv_indev_t *mousewheel = lv_sdl_mousewheel_create();
        lv_indev_t *keyboard = lv_sdl_keyboard_create();
        (void)mouse;
        (void)mousewheel;
        (void)keyboard;

        lv_obj_t *screen = lv_screen_active();
        if (!screen)
        {
            SDL_Quit();
            return;
        }

        lv_obj_t *tabview = lv_tabview_create(screen);
        lv_obj_set_size(tabview, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_pad_all(tabview, 0, 0);
        lv_tabview_set_tab_bar_size(tabview, TAB_BAR_HEIGHT);

        lv_obj_t *tab_bar = lv_tabview_get_tab_bar(tabview);
        if (tab_bar)
            lv_obj_set_style_text_font(tab_bar, &lv_font_montserrat_24, 0);

        lv_obj_t *tab_home = lv_tabview_add_tab(tabview, "Home");
        lv_obj_t *tab_diag = lv_tabview_add_tab(tabview, "Diagnostics");
        lv_obj_t *tab_debug = lv_tabview_add_tab(tabview, "Debug");

        UIWidgets widgets = {};
        widgets.tabview = tabview;
        build_home(tab_home, widgets, ui_state);
        build_diagnostics(tab_diag, widgets);
        build_debug(tab_debug, widgets);

        lv_image_dsc_t img_dsc = {};
        std::vector<uint8_t> img_copy;
        int diag_frame = 0;

        bool running = true;
        const int delay_ms = 5;
        while (running)
        {
            if (ui_state->quit_requested.load())
                break;

            SDL_Event event;
            if (SDL_PollEvent(&event))
            {
                if (event.type == SDL_QUIT ||
                    (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE))
                {
                    running = false;
                }
                else
                {
                    SDL_PushEvent(&event);
                }
            }

            ui_state->quit_requested.store(!running);
            if (!running)
                break;

            uint32_t active_tab = lv_tabview_get_tab_active(tabview);

            update_home(widgets, ui_state);

            if (active_tab == 1)
            {
                update_health_rows(widgets, ui_state);
                if (++diag_frame >= DIAG_REBUILD_INTERVAL)
                {
                    diag_frame = 0;
                    rebuild_diag_sections(widgets, ui_state);
                }
            }
            else
            {
                diag_frame = DIAG_REBUILD_INTERVAL;
            }

            if (active_tab == 2)
                update_debug(widgets, ui_state, img_dsc, img_copy);

            lv_tick_inc(delay_ms);
            lv_timer_handler();
            SDL_Delay(delay_ms);
        }

        lv_sdl_quit();
        SDL_Quit();
    }
} // namespace auto_battlebot
