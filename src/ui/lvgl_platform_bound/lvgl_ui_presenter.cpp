#include "lvgl_platform_bound/lvgl_ui_presenter.hpp"

#include <cstdio>

namespace auto_battlebot::ui_internal {

void derive_robot_counts(const RobotDescriptionsStamped &robots, bool &our_seen, int &opp_count) {
    our_seen = false;
    opp_count = 0;
    for (const auto &r : robots.descriptions) {
        if (r.is_stale) continue;
        if (r.group == Group::OURS) our_seen = true;
        if (r.group == Group::THEIRS) opp_count++;
    }
}

void update_rate_history(std::deque<double> &rate_history, int window, double current_hz) {
    if (window <= 0) window = 1;
    rate_history.push_back(current_hz);
    while (static_cast<int>(rate_history.size()) > window) rate_history.pop_front();
}

double get_rate_avg(const std::deque<double> &rate_history, double fallback_hz) {
    if (rate_history.empty()) return fallback_hz;
    double sum = 0.0;
    for (double v : rate_history) sum += v;
    return sum / static_cast<double>(rate_history.size());
}

bool compute_loop_met_sustained(
    std::optional<std::chrono::steady_clock::time_point> &rate_below_since, double avg_hz,
    double max_hz, double fail_threshold_fraction, double fail_duration_sec) {
    const double threshold_hz = max_hz * fail_threshold_fraction;
    const auto now = std::chrono::steady_clock::now();

    if (avg_hz >= threshold_hz) {
        rate_below_since.reset();
        return true;
    }
    if (!rate_below_since.has_value()) rate_below_since = now;
    const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - *rate_below_since).count();
    return elapsed < fail_duration_sec;
}

HomeViewModel present_home(const SystemStatus &st, bool our_seen, int opp_count,
                           int current_selected_opponent_count, double avg_hz, bool loop_met) {
    HomeViewModel vm;
    vm.selected_opponent_count = current_selected_opponent_count;
    if (st.selected_opponent_count >= 1 && st.selected_opponent_count <= 3) {
        vm.selected_opponent_count = st.selected_opponent_count;
    }

    const bool hardware_ok = st.camera_ok && st.transmitter_connected;
    const bool needs_reinit = !st.initialized;
    const bool has_fault = !hardware_ok || !loop_met;
    const bool tracking = our_seen && opp_count >= 1;

    if (has_fault) {
        vm.status_tile_color = 0xFF1744;
    } else if (!needs_reinit && tracking) {
        vm.status_tile_color = 0x00C853;
    } else {
        vm.status_tile_color = 0xFFC107;
    }

    if (!hardware_ok) {
        vm.status_label_text = "Hardware Disconnected";
    } else if (!loop_met) {
        vm.status_label_text = "Loop Rate Low";
    } else if (needs_reinit) {
        vm.status_label_text = "Press Reinitialize";
    } else {
        vm.status_label_text = "System Ready";
    }
    vm.status_label_text_color = has_fault ? 0xFFFFFF : 0x212121;
    vm.status_detail_text_color = has_fault ? 0xEEEEEE : 0x424242;

    char buf[256];
    if (!hardware_ok || !loop_met) {
        std::string details;
        if (!st.camera_ok) details += "Camera disconnected  ";
        if (!st.transmitter_connected) details += "Transmitter disconnected  ";
        if (!loop_met) {
            char rate_buf[64];
            snprintf(rate_buf, sizeof(rate_buf), "Loop slow (%.1f Hz)  ", avg_hz);
            details += rate_buf;
        }
        if (needs_reinit && hardware_ok) details += "Press Reinitialize Field  ";
        snprintf(buf, sizeof(buf), "%s", details.c_str());
    } else if (needs_reinit) {
        snprintf(buf, sizeof(buf), "Press Reinitialize Field to start");
    } else {
        snprintf(buf, sizeof(buf), "%.1f Hz  |  %s  |  %d opponent%s seen", avg_hz,
                 our_seen ? "our robot in view" : "our robot not seen", opp_count,
                 opp_count != 1 ? "s" : "");
    }
    vm.status_detail_text = buf;

    vm.reinit_tile_color = 0x00C853;
    if (!st.initialized) vm.reinit_tile_color = hardware_ok ? 0xFFC107 : 0xFF1744;
    if (st.initialized) {
        vm.reinit_status_text = "field initialized";
    } else if (hardware_ok) {
        vm.reinit_status_text = "press to initialize field";
    } else {
        vm.reinit_status_text = "reconnect camera/transmitter";
    }
    vm.reinit_should_pulse = hardware_ok && !st.initialized;

    vm.autonomy_tile_color = st.autonomy_enabled ? 0x00C853 : 0xFF1744;
    vm.autonomy_label_text = st.autonomy_enabled ? "Autonomy\nON" : "Autonomy\nOFF";
    vm.autonomy_label_text_color = st.autonomy_enabled ? 0x212121 : 0xFFFFFF;

    return vm;
}

SystemViewModel present_system(const SystemStatus &st) {
    SystemViewModel vm;
    vm.recording_tile_color = st.recording_enabled ? 0x00C853 : 0xFF1744;
    vm.recording_label_text = st.recording_enabled ? "Recording\nON" : "Recording\nOFF";
    vm.recording_label_text_color = st.recording_enabled ? 0x212121 : 0xFFFFFF;
    char buf[96];
    snprintf(buf, sizeof(buf), "SVO %s | MCAP %s", st.svo_recording_enabled ? "ON" : "OFF",
             st.mcap_recording_enabled ? "ON" : "OFF");
    vm.recording_detail_text = buf;
    vm.recording_detail_text_color = st.recording_enabled ? 0x212121 : 0xFAFAFA;
    return vm;
}

std::vector<HealthRowViewModel> present_health(const SystemStatus &st, bool our_seen, int opp_count,
                                               double avg_hz, bool loop_met) {
    std::vector<HealthRowViewModel> rows(8);
    char buf[128];

    snprintf(buf, sizeof(buf), "Camera: %s", st.camera_ok ? "Connected" : "Disconnected");
    rows[0] = {.visible = true, .ok = st.camera_ok, .text = buf};

    snprintf(buf, sizeof(buf), "Transmitter: %s",
             st.transmitter_connected ? "Connected" : "Disconnected");
    rows[1] = {.visible = true, .ok = st.transmitter_connected, .text = buf};

    if (st.initialized) {
        snprintf(buf, sizeof(buf), "Field: Initialized");
    } else if (st.camera_ok && st.transmitter_connected) {
        snprintf(buf, sizeof(buf), "Field: Press Reinitialize");
    } else {
        snprintf(buf, sizeof(buf), "Field: Waiting for camera/transmitter");
    }
    rows[2] = {.visible = true,
               .ok = st.initialized || (st.camera_ok && st.transmitter_connected),
               .text = buf};

    snprintf(buf, sizeof(buf), "Our Robot Seen: %s", our_seen ? "Yes" : "No");
    rows[3] = {.visible = true, .ok = our_seen, .text = buf};

    snprintf(buf, sizeof(buf), "Opponents Seen: %d", opp_count);
    rows[4] = {.visible = true, .ok = opp_count > 0, .text = buf};

    snprintf(buf, sizeof(buf), "Loop Rate: %.1f Hz %s", avg_hz, loop_met ? "(met)" : "(NOT MET)");
    rows[5] = {.visible = true, .ok = loop_met, .text = buf};

    if (st.jetson_temperature_c > 0.0) {
        snprintf(buf, sizeof(buf), "Jetson Temp: %.1f C", st.jetson_temperature_c);
        rows[6] = {.visible = true, .ok = st.jetson_temperature_c < 80.0, .text = buf};
    } else {
        rows[6] = {.visible = false, .ok = true, .text = ""};
    }

    if (!st.jetson_compute_mode.empty()) {
        snprintf(buf, sizeof(buf), "Compute Mode: %s", st.jetson_compute_mode.c_str());
        rows[7] = {.visible = true, .ok = true, .text = buf};
    } else {
        rows[7] = {.visible = false, .ok = true, .text = ""};
    }

    return rows;
}

std::string diag_section_key(const DiagnosticStatusSnapshot &snap) {
    std::string key = snap.name;
    if (!snap.subsection.empty()) key += "\x01" + snap.subsection;
    return key;
}

void merge_diag_snapshots(
    std::vector<std::string> &diag_section_order,
    std::map<std::string,
             std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        &diag_section_cache,
    const std::vector<DiagnosticStatusSnapshot> &snaps, std::chrono::steady_clock::time_point now) {
    for (const auto &snap : snaps) {
        const std::string key = diag_section_key(snap);
        auto it = diag_section_cache.find(key);
        if (it == diag_section_cache.end()) {
            diag_section_order.push_back(key);
            diag_section_cache[key] = {snap, now};
        } else {
            it->second.first = snap;
            it->second.second = now;
        }
    }
}

std::vector<DiagnosticsSectionViewModel> present_diagnostics_sections(
    const std::vector<std::string> &diag_section_order,
    const std::map<std::string,
                   std::pair<DiagnosticStatusSnapshot, std::chrono::steady_clock::time_point>>
        &diag_section_cache,
    std::chrono::steady_clock::time_point now, double stale_sec) {
    std::vector<DiagnosticsSectionViewModel> sections;
    sections.reserve(diag_section_order.size());
    for (const std::string &key : diag_section_order) {
        auto it = diag_section_cache.find(key);
        if (it == diag_section_cache.end()) continue;

        const DiagnosticStatusSnapshot &snap = it->second.first;
        const auto &updated = it->second.second;
        const double age_sec = std::chrono::duration<double>(now - updated).count();

        DiagnosticsSectionViewModel section;
        section.stale = (age_sec > stale_sec);
        section.title = snap.name;
        if (!snap.subsection.empty()) section.title += ": " + snap.subsection;
        if (!snap.message.empty()) section.rows.push_back({"message", snap.message});
        for (const auto &[k, v] : snap.values) section.rows.push_back({k, v});
        sections.push_back(std::move(section));
    }
    return sections;
}

}  // namespace auto_battlebot::ui_internal
