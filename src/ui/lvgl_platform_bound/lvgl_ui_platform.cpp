#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <lvgl.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stop_token>
#include <string>

#include "lvgl_platform_bound/lvgl_ui_battery.hpp"
#include "lvgl_platform_bound/lvgl_ui_composition.hpp"
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot {

void run_ui_thread(std::stop_token stop, std::shared_ptr<UIState> ui_state) {
    if (!ui_state) return;
    std::stop_callback on_stop(stop, [&ui_state] { ui_state->quit_requested.store(true); });

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        spdlog::error("UI thread failed to initialize SDL.");
        return;
    }

    int window_w = 1024;
    int window_h = 600;
    ui_state->get_window_size(window_w, window_h);
    if (window_w <= 0 || window_h <= 0) {
        spdlog::error("Configured window size is invalid. Using default size.");
        window_w = 1024;
        window_h = 600;
    }

    lv_init();
    lv_display_t *disp =
        lv_sdl_window_create(static_cast<int32_t>(window_w), static_cast<int32_t>(window_h));
    if (!disp) {
        SDL_Quit();
        return;
    }
    lv_sdl_window_set_title(disp, "Auto BattleBot");

    if (ui_state->get_fullscreen()) {
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

    lv_obj_t *root = lv_obj_create(screen);
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_style_pad_gap(root, 0, 0);
    lv_obj_set_style_radius(root, 0, 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);

    ui_internal::UIWidgets widgets = {};
    ui_internal::UiFrameState frame_state = {};
    ui_internal::UiServices services = {};
    const std::string battery_source =
        ui_internal::normalize_battery_source(ui_state->get_battery_source());
    services.battery_source =
        ui_internal::make_battery_source(battery_source, ui_state->get_battery_options());
    services.debug_overlay_renderer = ui_internal::make_debug_overlay_renderer();
    ui_internal::build_ui_content(root, widgets, ui_state, frame_state);

    bool running = true;
    const int delay_ms = 5;
    constexpr double kUiLoopWarnMs = 120.0;
    const auto heartbeat_interval = std::chrono::seconds(2);
    auto heartbeat_start = std::chrono::steady_clock::now();
    uint64_t loop_count = 0;
    double max_loop_ms = 0.0;
    while (running) {
        const auto loop_start = std::chrono::steady_clock::now();
        if (ui_state->quit_requested.load()) {
            spdlog::warn("UI thread observed quit_requested=true, exiting UI loop.");
            break;
        }

        SDL_Event event;
        if (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT ||
                (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE)) {
                running = false;
            } else {
                SDL_PushEvent(&event);
            }
        }

        if (!running) {
            if (widgets.controller) {
                widgets.controller->request_quit();
            } else {
                spdlog::warn("UI controller unavailable while handling window close.");
            }
        }
        if (!running) break;

        ui_internal::update_ui_content(widgets, ui_state, frame_state, services);

        lv_tick_inc(delay_ms);
        lv_timer_handler();
        SDL_Delay(delay_ms);

        const double loop_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - loop_start)
                .count();
        max_loop_ms = std::max(max_loop_ms, loop_ms);
        if (loop_ms > kUiLoopWarnMs) {
            spdlog::warn("UI loop iteration slow: elapsed_ms={:.2f}", loop_ms);
        }

        loop_count++;
        const auto now = std::chrono::steady_clock::now();
        if ((now - heartbeat_start) >= heartbeat_interval) {
            spdlog::debug("UI heartbeat: loops={} max_loop_ms={:.2f}", loop_count, max_loop_ms);
            heartbeat_start = now;
            loop_count = 0;
            max_loop_ms = 0.0;
        }
    }

    lv_sdl_quit();
    SDL_Quit();
}

}  // namespace auto_battlebot
