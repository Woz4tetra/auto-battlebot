#include "ui/ui_state.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

#include "data_structures/target_selection.hpp"

namespace auto_battlebot {
namespace {
constexpr double kDebugImageLockWaitWarnMs = 10.0;
constexpr double kDebugImageTotalWarnMs = 30.0;
}  // namespace

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

void UIState::set_debug_image(const cv::Mat &image) {
    // Clone outside the lock: the producer's only memcpy. Required because the camera SDK
    // reuses its backing buffer between captures, so we cannot safely share a refcount with it.
    const auto clone_start = std::chrono::steady_clock::now();
    cv::Mat fresh = image.clone();
    const double clone_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - clone_start)
            .count();

    const auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(status_mutex_);
    const double lock_wait_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start)
            .count();
    // cv::Mat assignment is a refcount swap; the previous Mat stays alive for any reader still
    // holding a refcount-shared view from get_debug_image().
    debug_image_ = std::move(fresh);
    const double total_ms = lock_wait_ms + std::chrono::duration<double, std::milli>(
                                               std::chrono::steady_clock::now() - lock_start)
                                               .count();
    if (lock_wait_ms > kDebugImageLockWaitWarnMs || total_ms > kDebugImageTotalWarnMs ||
        clone_ms > kDebugImageTotalWarnMs) {
        spdlog::warn(
            "validation: ui_state_set_debug_image slow clone_ms={:.2f} lock_wait_ms={:.2f} "
            "total_ms={:.2f} bytes={}",
            clone_ms, lock_wait_ms, total_ms, image.total() * image.elemSize());
    }
}

cv::Mat UIState::get_debug_image() const {
    const auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(status_mutex_);
    const double lock_wait_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start)
            .count();
    // Refcount-shared view; no memcpy.
    cv::Mat out = debug_image_;
    const double total_ms = lock_wait_ms + std::chrono::duration<double, std::milli>(
                                               std::chrono::steady_clock::now() - lock_start)
                                               .count();
    if (lock_wait_ms > kDebugImageLockWaitWarnMs || total_ms > kDebugImageTotalWarnMs) {
        spdlog::warn(
            "validation: ui_state_get_debug_image slow lock_wait_ms={:.2f} total_ms={:.2f} "
            "bytes={}",
            lock_wait_ms, total_ms, out.total() * out.elemSize());
    }
    return out;
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

void UIState::set_battery_source(const std::string &battery_source) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    battery_source_ = battery_source;
}

std::string UIState::get_battery_source() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return battery_source_;
}

void UIState::set_battery_options(const BatteryOptions &options) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    battery_options_ = options;
}

BatteryOptions UIState::get_battery_options() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return battery_options_;
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
