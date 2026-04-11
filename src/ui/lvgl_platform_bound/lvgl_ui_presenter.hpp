#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "ui/ui_state.hpp"

namespace auto_battlebot::ui_internal {

struct HomeViewModel {
    int selected_opponent_count = 1;
    uint32_t status_tile_color = 0xFFC107;
    std::string status_label_text;
    uint32_t status_label_text_color = 0x212121;
    std::string status_detail_text;
    uint32_t status_detail_text_color = 0x424242;
    uint32_t reinit_tile_color = 0x00C853;
    std::string reinit_status_text;
    bool reinit_should_pulse = false;
    uint32_t autonomy_tile_color = 0xFF1744;
    std::string autonomy_label_text;
    uint32_t autonomy_label_text_color = 0xFFFFFF;
};

struct SystemViewModel {
    uint32_t recording_tile_color = 0xFF1744;
    std::string recording_label_text;
    uint32_t recording_label_text_color = 0xFFFFFF;
    std::string recording_detail_text;
    uint32_t recording_detail_text_color = 0xFAFAFA;
};

struct HealthRowViewModel {
    bool visible = true;
    bool ok = false;
    std::string text;
};

struct DiagnosticsSectionViewModel {
    std::string title;
    bool stale = false;
    std::vector<std::pair<std::string, std::string>> rows;
};

void derive_robot_counts(const RobotDescriptionsStamped &robots, bool &our_seen, int &opp_count);
void update_rate_history(std::deque<double> &rate_history, int window, double current_hz);
double get_rate_avg(const std::deque<double> &rate_history, double fallback_hz);
bool compute_loop_met_sustained(
    std::optional<std::chrono::steady_clock::time_point> &rate_below_since, double avg_hz,
    double max_hz, double fail_threshold_fraction, double fail_duration_sec);

HomeViewModel present_home(const SystemStatus &st, bool our_seen, int opp_count,
                           int current_selected_opponent_count, double avg_hz, bool loop_met);
SystemViewModel present_system(const SystemStatus &st);
std::vector<HealthRowViewModel> present_health(const SystemStatus &st, bool our_seen, int opp_count,
                                               double avg_hz, bool loop_met);

std::string diag_section_key(const DiagnosticStatusSnapshot &snap);
void merge_diag_snapshots(
    std::vector<std::string> &diag_section_order,
    std::map<std::string,
             std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        &diag_section_cache,
    const std::vector<DiagnosticStatusSnapshot> &snaps, std::chrono::steady_clock::time_point now);
std::vector<DiagnosticsSectionViewModel> present_diagnostics_sections(
    const std::vector<std::string> &diag_section_order,
    const std::map<std::string,
                   std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        &diag_section_cache,
    std::chrono::steady_clock::time_point now, double stale_sec);

}  // namespace auto_battlebot::ui_internal
