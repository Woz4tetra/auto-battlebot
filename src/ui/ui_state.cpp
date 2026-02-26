#include "ui/ui_state.hpp"

namespace auto_battlebot
{
    void UIState::set_system_status(const SystemStatus &s)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        system_status_ = s;
    }

    void UIState::get_system_status(SystemStatus &out) const
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        out = system_status_;
    }

    void UIState::set_diagnostics(const std::map<std::string, std::string> &kv)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        diagnostics_ = kv;
    }

    void UIState::get_diagnostics(std::map<std::string, std::string> &out) const
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        out = diagnostics_;
    }

    void UIState::set_debug_image(int width, int height, int channels, const std::vector<uint8_t> &data)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        debug_image_width_ = width;
        debug_image_height_ = height;
        debug_image_channels_ = channels;
        debug_image_data_ = data;
    }

    void UIState::get_debug_image(int &width, int &height, int &channels, std::vector<uint8_t> &data) const
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        width = debug_image_width_;
        height = debug_image_height_;
        channels = debug_image_channels_;
        data = debug_image_data_;
    }

    void UIState::set_robots(const RobotDescriptionsStamped &robots)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        robots_ = robots;
    }

    void UIState::get_robots(RobotDescriptionsStamped &out) const
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        out = robots_;
    }

    void UIState::set_keypoints(const KeypointsStamped &keypoints)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        keypoints_ = keypoints;
    }

    void UIState::get_keypoints(KeypointsStamped &out) const
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        out = keypoints_;
    }
} // namespace auto_battlebot
