#include "ui/ui_state.hpp"
#include "enums/label.hpp"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace auto_battlebot
{
    namespace
    {
        constexpr float TOUCH_SCALE = 2.0f;
        constexpr int WINDOW_FLAGS = SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_RESIZABLE;

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

        void update_texture(GLuint &tex_id, int width, int height, int channels, const std::vector<uint8_t> &data)
        {
            if (width <= 0 || height <= 0 || data.empty())
                return;
            if (tex_id == 0)
                glGenTextures(1, &tex_id);
            glBindTexture(GL_TEXTURE_2D, tex_id);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            GLenum format = (channels == 3) ? GL_RGB : ((channels == 4) ? GL_RGBA : GL_RED);
            glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data.data());
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    } // namespace

    void run_ui_thread(std::shared_ptr<UIState> ui_state)
    {
        if (!ui_state)
            return;

        if (SDL_Init(SDL_INIT_VIDEO) != 0)
        {
            return;
        }

        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        SDL_Window *window = SDL_CreateWindow(
            "Auto BattleBot",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            1280, 720,
            WINDOW_FLAGS);
        if (!window)
        {
            SDL_Quit();
            return;
        }

        SDL_GLContext gl_context = SDL_GL_CreateContext(window);
        if (!gl_context)
        {
            SDL_DestroyWindow(window);
            SDL_Quit();
            return;
        }
        SDL_GL_MakeCurrent(window, gl_context);
        SDL_GL_SetSwapInterval(1);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO &io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        ImGui::StyleColorsDark();
        ImGui::GetStyle().ScaleAllSizes(TOUCH_SCALE);
        io.FontGlobalScale = TOUCH_SCALE;

        ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
        ImGui_ImplOpenGL3_Init("#version 130");

        GLuint debug_tex_id = 0;
        int last_width = 0, last_height = 0;

        bool running = true;
        while (running)
        {
            SDL_Event event;
            while (SDL_PollEvent(&event))
            {
                ImGui_ImplSDL2_ProcessEvent(&event);
                if (event.type == SDL_QUIT)
                    running = false;
                if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE)
                    running = false;
            }

            ui_state->quit_requested.store(!running);

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();

            SystemStatus status;
            ui_state->get_system_status(status);
            RobotDescriptionsStamped robots;
            ui_state->get_robots(robots);
            bool our_robot_seen = false;
            int opponent_count_seen = 0;
            derive_robot_counts(robots, our_robot_seen, opponent_count_seen);

            if (ImGui::Begin("System", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("Camera: %s", status.camera_ok ? "OK" : "FAIL");
                ImGui::Text("Transmitter: %s", status.transmitter_connected ? "Connected" : "Disconnected");
                ImGui::Text("Loop: %.1f Hz %s", status.loop_rate_hz, status.loop_met ? "(met)" : "");
                ImGui::Text("Initialized: %s", status.initialized ? "Yes" : "No");
                ImGui::Text("Our robot seen: %s", our_robot_seen ? "Yes" : "No");
                ImGui::Text("Opponents seen: %d", opponent_count_seen);

                if (ImGui::Button("Reinitialize field"))
                {
                    ui_state->reinit_requested.store(true);
                }
                ImGui::SameLine();
                if (ImGui::Button("1 opponent"))
                {
                    ui_state->opponent_count_requested.store(1);
                }
                ImGui::SameLine();
                if (ImGui::Button("2 opponents"))
                {
                    ui_state->opponent_count_requested.store(2);
                }
                ImGui::SameLine();
                if (ImGui::Button("3 opponents"))
                {
                    ui_state->opponent_count_requested.store(3);
                }
            }
            ImGui::End();

            std::map<std::string, std::string> diagnostics;
            ui_state->get_diagnostics(diagnostics);
            if (ImGui::Begin("Diagnostics", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                for (const auto &[k, v] : diagnostics)
                {
                    ImGui::Text("%s: %s", k.c_str(), v.c_str());
                }
            }
            ImGui::End();

            int dw = 0, dh = 0, dc = 0;
            std::vector<uint8_t> image_data;
            ui_state->get_debug_image(dw, dh, dc, image_data);
            KeypointsStamped keypoints;
            ui_state->get_keypoints(keypoints);

            if (dw > 0 && dh > 0 && !image_data.empty())
            {
                update_texture(debug_tex_id, dw, dh, dc, image_data);
                last_width = dw;
                last_height = dh;
            }

            if (ImGui::Begin("Debug view", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                if (debug_tex_id != 0 && last_width > 0 && last_height > 0)
                {
                    ImVec2 size(static_cast<float>(last_width), static_cast<float>(last_height));
                    ImVec2 uv0(0, 1), uv1(1, 0);
                    ImGui::Image(reinterpret_cast<ImTextureID>(static_cast<uintptr_t>(debug_tex_id)), size, uv0, uv1);

                    ImVec2 min_pos = ImGui::GetItemRectMin();
                    ImVec2 max_pos = ImGui::GetItemRectMax();
                    float iw = max_pos.x - min_pos.x;
                    float ih = max_pos.y - min_pos.y;
                    if (iw > 0 && ih > 0)
                    {
                        ImDrawList *draw_list = ImGui::GetWindowDrawList();
                        for (const auto &kp : keypoints.keypoints)
                        {
                            float u = static_cast<float>(kp.x) / static_cast<float>(last_width);
                            float v = static_cast<float>(kp.y) / static_cast<float>(last_height);
                            float px = min_pos.x + u * iw;
                            float py = min_pos.y + (1.0f - v) * ih;
                            draw_list->AddCircleFilled(ImVec2(px, py), 6.0f * TOUCH_SCALE, IM_COL32(0, 255, 0, 255));
                        }
                    }
                }
            }
            ImGui::End();

            ImGui::Render();
            glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            SDL_GL_SwapWindow(window);
        }

        if (debug_tex_id != 0)
            glDeleteTextures(1, &debug_tex_id);
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
} // namespace auto_battlebot
