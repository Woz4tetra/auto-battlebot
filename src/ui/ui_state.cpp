#include "ui/ui_state.hpp"

#include "target_selector/target_selector_interface.hpp"

namespace auto_battlebot {
void UIState::set_system_status(const SystemStatus &s) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    system_status_ = s;
}

void UIState::get_system_status(SystemStatus &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = system_status_;
}

void UIState::set_diagnostics(const std::map<std::string, std::string> &kv) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    diagnostics_ = kv;
}

void UIState::get_diagnostics(std::map<std::string, std::string> &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = diagnostics_;
}

void UIState::set_diagnostic_snapshots(const std::vector<DiagnosticStatusSnapshot> &snapshots) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    diagnostic_snapshots_ = snapshots;
}

void UIState::get_diagnostic_snapshots(std::vector<DiagnosticStatusSnapshot> &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = diagnostic_snapshots_;
}

void UIState::set_debug_image(int width, int height, int channels,
                              const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    debug_image_width_ = width;
    debug_image_height_ = height;
    debug_image_channels_ = channels;
    debug_image_data_ = data;
}

void UIState::get_debug_image(int &width, int &height, int &channels,
                              std::vector<uint8_t> &data) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    width = debug_image_width_;
    height = debug_image_height_;
    channels = debug_image_channels_;
    data = debug_image_data_;
}

void UIState::set_robots(const RobotDescriptionsStamped &robots) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    robots_ = robots;
}

void UIState::get_robots(RobotDescriptionsStamped &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = robots_;
}

void UIState::set_keypoints(const KeypointsStamped &keypoints) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    keypoints_ = keypoints;
}

void UIState::get_keypoints(KeypointsStamped &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = keypoints_;
}

void UIState::set_navigation_path(const std::optional<NavigationPathSegment> &path) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    navigation_path_ = path;
}

void UIState::get_navigation_path(std::optional<NavigationPathSegment> &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = navigation_path_;
}

void UIState::set_manual_target(const std::optional<TargetSelection> &target) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    manual_target_ = target;
}

std::optional<TargetSelection> UIState::get_manual_target() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return manual_target_;
}

void UIState::set_field_description(const std::optional<FieldDescription> &field) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    field_description_ = field;
}

void UIState::get_field_description(std::optional<FieldDescription> &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = field_description_;
}

void UIState::set_camera_info(const CameraInfo &camera_info) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    camera_info_ = camera_info;
}

void UIState::get_camera_info(CameraInfo &out) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    out = camera_info_;
}

void UIState::set_window_size(int width, int height) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    window_width_ = width;
    window_height_ = height;
}

void UIState::set_fullscreen(bool fullscreen) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    fullscreen_ = fullscreen;
}

bool UIState::get_fullscreen() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return fullscreen_;
}

void UIState::get_window_size(int &width, int &height) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    width = window_width_;
    height = window_height_;
}

void UIState::set_rate_avg_window(int window) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    rate_avg_window_ = window <= 0 ? 1 : window;
}

int UIState::get_rate_avg_window() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return rate_avg_window_;
}

void UIState::set_max_loop_rate(double hz) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    max_loop_rate_hz_ = hz > 0.0 ? hz : 300.0;
}

double UIState::get_max_loop_rate() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return max_loop_rate_hz_;
}

void UIState::set_rate_fail_threshold(double fraction) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    rate_fail_threshold_ = (fraction > 0.0 && fraction <= 1.0) ? fraction : 0.99;
}

double UIState::get_rate_fail_threshold() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return rate_fail_threshold_;
}

void UIState::set_rate_fail_duration_sec(double sec) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    rate_fail_duration_sec_ = sec >= 0.0 ? sec : 2.0;
}

double UIState::get_rate_fail_duration_sec() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return rate_fail_duration_sec_;
}
}  // namespace auto_battlebot
