#define SDL_MAIN_HANDLED
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"
#include "enums/label.hpp"
#include <SDL.h>
#include <lvgl.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace auto_battlebot
{
    namespace
    {
        constexpr int TOUCH_SCALE_PERCENT = 200; /* 2x for touch-friendly */

        void derive_robot_counts(const RobotDescriptionsStamped &robots, bool &our_seen, int &opponent_count)
        {
            our_seen = false;
            opponent_count = 0;
            for (const auto &r : robots.descriptions)
            {
                if (r.label == Label::MR_STABS_MK1 || r.label == Label::MR_STABS_MK2 ||
                    r.label == Label::MRS_BUFF_MK1 || r.label == Label::MRS_BUFF_MK2)
                    our_seen = true;
                if (r.label == Label::OPPONENT || r.label == Label::HOUSE_BOT)
                    opponent_count++;
            }
        }

        struct OpponentBtnData
        {
            std::shared_ptr<UIState> ui_state;
            int count;
        };

        void reinit_clicked_cb(lv_event_t *e)
        {
            void *ud = lv_event_get_user_data(e);
            if (ud)
                static_cast<UIState *>(ud)->reinit_requested.store(true);
        }

        void opponent_clicked_cb(lv_event_t *e)
        {
            void *ud = lv_event_get_user_data(e);
            if (ud)
            {
                auto *data = static_cast<OpponentBtnData *>(ud);
                if (data->ui_state)
                    data->ui_state->opponent_count_requested.store(data->count);
            }
        }

        lv_display_t *create_ui(
            std::shared_ptr<UIState> ui_state,
            lv_obj_t *&system_cont,
            lv_obj_t *&diag_cont,
            lv_obj_t *&debug_img,
            lv_obj_t *&debug_keypoints_cont,
            std::vector<lv_obj_t *> &keypoint_dots)
        {
            lv_obj_t *screen = lv_screen_active();
            if (!screen)
                return nullptr;

            lv_obj_t *tabview = lv_tabview_create(screen);
            lv_obj_set_size(tabview, LV_PCT(100), LV_PCT(100));
            lv_obj_set_style_pad_all(tabview, 4, 0);

            lv_obj_t *tab_system = lv_tabview_add_tab(tabview, "System");
            lv_obj_t *tab_diag = lv_tabview_add_tab(tabview, "Diagnostics");
            lv_obj_t *tab_debug = lv_tabview_add_tab(tabview, "Debug");

            /* System tab: labels + buttons */
            system_cont = lv_obj_create(tab_system);
            lv_obj_set_size(system_cont, LV_PCT(100), LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(system_cont, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(system_cont, 8, 0);
            lv_obj_set_style_pad_gap(system_cont, 4, 0);
            lv_obj_set_scrollbar_mode(system_cont, LV_SCROLLBAR_MODE_AUTO);

            /* Placeholder labels - we update text each frame */
            for (int i = 0; i < 6; i++)
                lv_label_create(system_cont);

            static std::shared_ptr<UIState> s_ui_state;
            s_ui_state = ui_state;
            static OpponentBtnData opp_btn_data[3];

            lv_obj_t *btn_reinit = lv_button_create(system_cont);
            lv_obj_t *lbl_reinit = lv_label_create(btn_reinit);
            lv_label_set_text(lbl_reinit, "Reinitialize field");
            lv_obj_center(lbl_reinit);
            lv_obj_add_event_cb(btn_reinit, reinit_clicked_cb, LV_EVENT_CLICKED, ui_state.get());

            lv_obj_t *row_opp = lv_obj_create(system_cont);
            lv_obj_set_size(row_opp, LV_PCT(100), LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(row_opp, LV_FLEX_FLOW_ROW);
            lv_obj_set_style_pad_gap(row_opp, 4, 0);
            lv_obj_set_style_pad_all(row_opp, 0, 0);
            lv_obj_clear_flag(row_opp, LV_OBJ_FLAG_SCROLLABLE);
            for (int i = 1; i <= 3; i++)
            {
                opp_btn_data[i - 1] = {ui_state, i};
                lv_obj_t *btn = lv_button_create(row_opp);
                lv_obj_t *lbl = lv_label_create(btn);
                lv_label_set_text_fmt(lbl, "%d opponent(s)", i);
                lv_obj_center(lbl);
                lv_obj_add_event_cb(btn, opponent_clicked_cb, LV_EVENT_CLICKED, &opp_btn_data[i - 1]);
            }

            /* Diagnostics tab: scrollable list of key-value */
            diag_cont = lv_obj_create(tab_diag);
            lv_obj_set_size(diag_cont, LV_PCT(100), LV_PCT(100));
            lv_obj_set_flex_flow(diag_cont, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(diag_cont, 8, 0);
            lv_obj_set_style_pad_gap(diag_cont, 2, 0);
            lv_obj_set_scrollbar_mode(diag_cont, LV_SCROLLBAR_MODE_AUTO);

            /* Debug tab: image + keypoints overlay */
            lv_obj_t *debug_cont = lv_obj_create(tab_debug);
            lv_obj_set_size(debug_cont, LV_PCT(100), LV_PCT(100));
            lv_obj_set_flex_flow(debug_cont, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(debug_cont, 4, 0);
            lv_obj_clear_flag(debug_cont, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *img_cont = lv_obj_create(debug_cont);
            lv_obj_set_size(img_cont, LV_PCT(100), LV_PCT(100));
            lv_obj_set_flex_grow(img_cont, 1);
            lv_obj_clear_flag(img_cont, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_pad_all(img_cont, 0, 0);

            debug_img = lv_image_create(img_cont);
            lv_obj_align(debug_img, LV_ALIGN_TOP_LEFT, 0, 0);

            debug_keypoints_cont = lv_obj_create(img_cont);
            lv_obj_set_size(debug_keypoints_cont, LV_PCT(100), LV_PCT(100));
            lv_obj_align(debug_keypoints_cont, LV_ALIGN_TOP_LEFT, 0, 0);
            lv_obj_clear_flag(debug_keypoints_cont, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_bg_opa(debug_keypoints_cont, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(debug_keypoints_cont, 0, 0);
            lv_obj_set_style_pad_all(debug_keypoints_cont, 0, 0);
            lv_obj_set_style_radius(debug_keypoints_cont, 0, 0);

            keypoint_dots.clear();
            return lv_obj_get_display(screen);
        }

        void update_system_panel(lv_obj_t *system_cont, std::shared_ptr<UIState> ui_state)
        {
            if (!system_cont || !ui_state)
                return;
            SystemStatus status;
            ui_state->get_system_status(status);
            RobotDescriptionsStamped robots;
            ui_state->get_robots(robots);
            bool our_seen = false;
            int opponent_count = 0;
            derive_robot_counts(robots, our_seen, opponent_count);

            lv_obj_t *lbl0 = lv_obj_get_child(system_cont, 0);
            if (lbl0)
                lv_label_set_text_fmt(lbl0, "Camera: %s", status.camera_ok ? "OK" : "FAIL");
            lv_obj_t *lbl1 = lv_obj_get_child(system_cont, 1);
            if (lbl1)
                lv_label_set_text_fmt(lbl1, "Transmitter: %s", status.transmitter_connected ? "Connected" : "Disconnected");
            lv_obj_t *lbl2 = lv_obj_get_child(system_cont, 2);
            if (lbl2)
                lv_label_set_text_fmt(lbl2, "Loop: %.1f Hz %s", status.loop_rate_hz, status.loop_met ? "(met)" : "");
            lv_obj_t *lbl3 = lv_obj_get_child(system_cont, 3);
            if (lbl3)
                lv_label_set_text_fmt(lbl3, "Initialized: %s", status.initialized ? "Yes" : "No");
            lv_obj_t *lbl4 = lv_obj_get_child(system_cont, 4);
            if (lbl4)
                lv_label_set_text_fmt(lbl4, "Our robot seen: %s", our_seen ? "Yes" : "No");
            lv_obj_t *lbl5 = lv_obj_get_child(system_cont, 5);
            if (lbl5)
                lv_label_set_text_fmt(lbl5, "Opponents seen: %d", opponent_count);
        }

        void update_diagnostics_panel(lv_obj_t *diag_cont, std::shared_ptr<UIState> ui_state)
        {
            if (!diag_cont || !ui_state)
                return;
            std::map<std::string, std::string> diagnostics;
            ui_state->get_diagnostics(diagnostics);
            uint32_t n = lv_obj_get_child_count(diag_cont);
            size_t need = diagnostics.size();
            while (n < need)
            {
                lv_label_create(diag_cont);
                n++;
            }
            size_t i = 0;
            for (const auto &[k, v] : diagnostics)
            {
                lv_obj_t *lbl = lv_obj_get_child(diag_cont, static_cast<int32_t>(i));
                if (lbl)
                {
                    char buf[256];
                    snprintf(buf, sizeof(buf), "%s: %s", k.c_str(), v.c_str());
                    lv_label_set_text(lbl, buf);
                }
                i++;
            }
        }

        void update_debug_image_and_keypoints(
            lv_obj_t *img_widget,
            lv_obj_t *keypoints_cont,
            std::vector<lv_obj_t *> &keypoint_dots,
            std::shared_ptr<UIState> ui_state,
            lv_image_dsc_t &img_dsc,
            std::vector<uint8_t> &img_copy)
        {
            if (!img_widget || !ui_state)
                return;
            int dw = 0, dh = 0, dc = 0;
            std::vector<uint8_t> image_data;
            ui_state->get_debug_image(dw, dh, dc, image_data);
            KeypointsStamped keypoints;
            ui_state->get_keypoints(keypoints);

            if (dw > 0 && dh > 0 && !image_data.empty())
            {
                size_t expected = static_cast<size_t>(dw) * dh * (dc == 4 ? 4 : 3);
                if (image_data.size() >= expected)
                {
                    img_copy = image_data;
                    img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
                    img_dsc.header.cf = (dc == 4) ? LV_COLOR_FORMAT_XRGB8888 : LV_COLOR_FORMAT_RGB888;
                    img_dsc.header.flags = 0;
                    img_dsc.header.w = static_cast<uint32_t>(dw);
                    img_dsc.header.h = static_cast<uint32_t>(dh);
                    img_dsc.header.stride = dw * (dc == 4 ? 4 : 3);
                    img_dsc.data_size = static_cast<uint32_t>(img_copy.size());
                    img_dsc.data = img_copy.data();
                    lv_image_set_src(img_widget, &img_dsc);
                    lv_obj_set_size(img_widget, dw, dh);
                    lv_obj_set_size(keypoints_cont, dw, dh);
                }
            }

            /* Keypoint dots overlay */
            uint32_t need_dots = static_cast<uint32_t>(keypoints.keypoints.size());
            while (keypoint_dots.size() < need_dots)
            {
                lv_obj_t *dot = lv_obj_create(keypoints_cont);
                lv_obj_set_size(dot, 12, 12);
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                lv_obj_set_style_bg_color(dot, lv_color_hex(0x00FF00), 0);
                lv_obj_set_style_border_width(dot, 0, 0);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
                keypoint_dots.push_back(dot);
            }
            int32_t img_w = lv_obj_get_width(img_widget);
            int32_t img_h = lv_obj_get_height(img_widget);
            if (img_w <= 0)
                img_w = 1;
            if (img_h <= 0)
                img_h = 1;
            for (size_t i = 0; i < keypoint_dots.size(); i++)
            {
                lv_obj_t *dot = keypoint_dots[i];
                if (i < keypoints.keypoints.size())
                {
                    const auto &kp = keypoints.keypoints[i];
                    int32_t px = static_cast<int32_t>(kp.x) - 6;
                    int32_t py = static_cast<int32_t>(kp.y) - 6;
                    lv_obj_set_pos(dot, px, py);
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
        lv_indev_t *mouse = lv_sdl_mouse_create();
        lv_indev_t *mousewheel = lv_sdl_mousewheel_create();
        lv_indev_t *keyboard = lv_sdl_keyboard_create();
        (void)mouse;
        (void)mousewheel;
        (void)keyboard;

        lv_obj_t *system_cont = nullptr;
        lv_obj_t *diag_cont = nullptr;
        lv_obj_t *debug_img = nullptr;
        lv_obj_t *debug_keypoints_cont = nullptr;
        std::vector<lv_obj_t *> keypoint_dots;
        create_ui(ui_state, system_cont, diag_cont, debug_img, debug_keypoints_cont, keypoint_dots);

        lv_image_dsc_t img_dsc = {};
        std::vector<uint8_t> img_copy;

        bool running = true;
        const int delay_ms = 5;
        using Clock = std::chrono::steady_clock;
        auto last_loop_end = Clock::now();
        double prev_system_ms = 0, prev_diag_ms = 0, prev_debug_ms = 0;
        uint64_t iter = 0;
        while (running)
        {
            // #region agent log
            auto loop_start = Clock::now();
            double loop_duration_ms = (last_loop_end.time_since_epoch().count() != 0)
                ? 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(loop_start - last_loop_end).count()
                : 0;
            std::ofstream f("/tmp/auto_battlebot_ui_debug.log", std::ios::app);
            if (f) {
                f << "{\"hypothesisId\":\"H1_loop\",\"message\":\"slow_loop\",\"data\":{\"loop_duration_ms\":" << loop_duration_ms
                    << ",\"prev_system_ms\":" << prev_system_ms << ",\"prev_diag_ms\":" << prev_diag_ms << ",\"prev_debug_ms\":" << prev_debug_ms
                    << ",\"iter\":" << iter << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(loop_start.time_since_epoch()).count() << "}\n";
            }
            // #endregion
            if (ui_state->quit_requested.load())
                break;

            /* Only consume quit/close; push other events back so LVGL's SDL driver
             * (run inside lv_timer_handler) can process touch/mouse/keyboard. */
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

            // #region agent log
            auto t0 = Clock::now();
            // #endregion
            update_system_panel(system_cont, ui_state);
            // #region agent log
            auto t1 = Clock::now();
            prev_system_ms = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
            // #endregion
            update_diagnostics_panel(diag_cont, ui_state);
            // #region agent log
            auto t2 = Clock::now();
            prev_diag_ms = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
            // #endregion
            update_debug_image_and_keypoints(debug_img, debug_keypoints_cont, keypoint_dots, ui_state, img_dsc, img_copy);
            // #region agent log
            auto t3 = Clock::now();
            prev_debug_ms = 1e-6 * std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count();
            // #endregion

            lv_tick_inc(delay_ms);
            lv_timer_handler();
            SDL_Delay(delay_ms);
            last_loop_end = Clock::now();
            iter++;
        }

        lv_sdl_quit();
        SDL_Quit();
    }
} // namespace auto_battlebot
