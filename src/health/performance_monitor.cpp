#include "health/performance_monitor.hpp"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <magic_enum.hpp>
#include <numbers>
#include <utility>

namespace auto_battlebot {
namespace {

size_t index_of(MonitorLink link) { return static_cast<size_t>(link); }

std::string lower(std::string_view text) {
    std::string out(text);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

double wrap_angle(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }

std::optional<double> pearson(const std::vector<double> &a, const std::vector<double> &b) {
    const size_t n = a.size();
    if (n < 2) return std::nullopt;
    double mean_a = 0.0;
    double mean_b = 0.0;
    for (size_t i = 0; i < n; ++i) {
        mean_a += a[i];
        mean_b += b[i];
    }
    mean_a /= static_cast<double>(n);
    mean_b /= static_cast<double>(n);
    double cov = 0.0;
    double var_a = 0.0;
    double var_b = 0.0;
    for (size_t i = 0; i < n; ++i) {
        cov += (a[i] - mean_a) * (b[i] - mean_b);
        var_a += (a[i] - mean_a) * (a[i] - mean_a);
        var_b += (b[i] - mean_b) * (b[i] - mean_b);
    }
    if (var_a <= 0.0 || var_b <= 0.0) return std::nullopt;
    return cov / std::sqrt(var_a * var_b);
}

double standard_deviation(const std::vector<double> &values) {
    if (values.size() < 2) return 0.0;
    double mean = 0.0;
    for (double v : values) mean += v;
    mean /= static_cast<double>(values.size());
    double var = 0.0;
    for (double v : values) var += (v - mean) * (v - mean);
    return std::sqrt(var / static_cast<double>(values.size()));
}

struct Fraction {
    int hits = 0;
    int total = 0;
    double value() const { return total > 0 ? static_cast<double>(hits) / total : 0.0; }
};

}  // namespace

std::string check_name(PerformanceCheck check) { return lower(magic_enum::enum_name(check)); }

PerformanceCheck link_check(MonitorLink link) {
    switch (link) {
        case MonitorLink::CAMERA_FRAMES:
            return PerformanceCheck::CAMERA_FRAMES_LINK;
        case MonitorLink::PERCEPTION:
            return PerformanceCheck::PERCEPTION_LINK;
        case MonitorLink::FILTER:
            return PerformanceCheck::FILTER_LINK;
        case MonitorLink::TRANSMIT:
            return PerformanceCheck::TRANSMIT_LINK;
        case MonitorLink::CRSF_TELEMETRY:
            return PerformanceCheck::CRSF_TELEMETRY_LINK;
        case MonitorLink::ESPNOW:
            return PerformanceCheck::ESPNOW_LINK;
        case MonitorLink::IMU:
            return PerformanceCheck::IMU_LINK;
        case MonitorLink::RELAY_PUBLISH:
            return PerformanceCheck::RELAY_PUBLISH_LINK;
    }
    return PerformanceCheck::CAMERA_FRAMES_LINK;
}

PerformanceMonitor::PerformanceMonitor(PerformanceMonitorConfiguration config)
    : config_(std::move(config)) {}

void PerformanceMonitor::add_link_events(MonitorLink link, uint64_t count) {
    pushed_[index_of(link)].fetch_add(count, std::memory_order_relaxed);
}

void PerformanceMonitor::set_link_total(MonitorLink link, uint64_t total) {
    pushed_[index_of(link)].store(total, std::memory_order_relaxed);
}

void PerformanceMonitor::set_link_source(MonitorLink link, std::function<uint64_t()> source) {
    links_[index_of(link)].source = std::move(source);
}

bool PerformanceMonitor::has_link_source(MonitorLink link) const {
    return static_cast<bool>(links_[index_of(link)].source);
}

void PerformanceMonitor::set_link_expected(MonitorLink link, bool expected, double now) {
    LinkState &state = links_[index_of(link)];
    if (expected == state.expected) return;
    state.expected = expected;
    if (!expected) return;
    update_link(link, now);
    state.expected_since = now;
    state.last_change_s.reset();
    state.samples.clear();
    state.samples.emplace_back(now, state.total);
}

PerformanceMonitor::LinkLimits PerformanceMonitor::limits(MonitorLink link) const {
    switch (link) {
        case MonitorLink::CAMERA_FRAMES:
            return {config_.camera_frames_min_rate_hz, config_.camera_frames_max_age_s};
        case MonitorLink::PERCEPTION:
            return {config_.perception_min_rate_hz, config_.perception_max_age_s};
        case MonitorLink::FILTER:
            return {config_.filter_min_rate_hz, config_.filter_max_age_s};
        case MonitorLink::TRANSMIT:
            return {config_.transmit_min_rate_hz, config_.transmit_max_age_s};
        case MonitorLink::CRSF_TELEMETRY:
            return {config_.crsf_telemetry_min_rate_hz, config_.crsf_telemetry_max_age_s};
        case MonitorLink::ESPNOW:
            return {config_.espnow_min_rate_hz, config_.espnow_max_age_s};
        case MonitorLink::IMU:
            return {config_.imu_min_rate_hz, config_.imu_max_age_s};
        case MonitorLink::RELAY_PUBLISH:
            return {config_.relay_publish_min_rate_hz, config_.relay_publish_max_age_s};
    }
    return {0.0, 0.0};
}

void PerformanceMonitor::update_link(MonitorLink link, double now) {
    LinkState &state = links_[index_of(link)];
    const uint64_t total =
        state.source ? state.source() : pushed_[index_of(link)].load(std::memory_order_relaxed);
    if (total == state.total) return;
    state.total = total;
    state.last_change_s = now;
    if (state.expected) state.samples.emplace_back(now, total);
}

std::optional<double> PerformanceMonitor::link_rate(const LinkState &state, double now) const {
    if (!state.expected || now - state.expected_since < config_.window_s) return std::nullopt;
    const double boundary = now - config_.window_s;
    uint64_t baseline = state.samples.empty() ? state.total : state.samples.front().second;
    for (const auto &[t, total] : state.samples) {
        if (t > boundary) break;
        baseline = total;
    }
    return static_cast<double>(state.total - baseline) / config_.window_s;
}

double PerformanceMonitor::link_age(const LinkState &state, double now) const {
    return now - state.last_change_s.value_or(state.expected_since);
}

void PerformanceMonitor::record_tick(const MonitorTick &tick) {
    TickRecord record;
    record.t = tick.t;
    record.autonomy = tick.autonomy_driving;
    if (tick.autonomy_driving && tick.policy_linear && tick.sent_linear) {
        record.slew_bound =
            std::abs(*tick.policy_linear - *tick.sent_linear) > config_.slew_difference;
    }
    if (tick.guard_scale) record.guard_bound = *tick.guard_scale < config_.guard_scale_floor;
    if (tick.field && tick.our_pose) {
        record.outside_field = std::abs(tick.our_pose->x) > tick.field->half_x ||
                               std::abs(tick.our_pose->y) > tick.field->half_y;
    }
    record.our_present = tick.our_pose.has_value();
    if (tick.opponent_expected) record.opponent_present = tick.opponent_present;
    record.espnow_ack_percent = tick.espnow_ack_percent;
    record.command_age_ms = tick.command_age_ms;
    ticks_.push_back(record);

    std::map<std::string, double> still_since;
    std::map<std::string, double> still_sigma;
    for (const MonitorTrack &track : tick.tracks) {
        if (track.sigma_m <= config_.phantom_sigma_m ||
            track.speed_mps >= config_.phantom_still_speed_mps) {
            continue;
        }
        const auto previous = still_large_since_.find(track.name);
        still_since[track.name] = previous != still_large_since_.end() ? previous->second : tick.t;
        still_sigma[track.name] = track.sigma_m;
    }
    still_large_since_ = std::move(still_since);
    still_large_sigma_ = std::move(still_sigma);

    if (tick.imu_yaw_rate_ccw_radps) {
        imu_sum_ += *tick.imu_yaw_rate_ccw_radps;
        ++imu_count_;
    }
    if (tick.our_pose && tick.our_heading_measured) {
        if (last_heading_t_ && tick.t > *last_heading_t_ && imu_count_ > 0) {
            const double rate =
                wrap_angle(tick.our_pose->yaw - last_heading_) / (tick.t - *last_heading_t_);
            yaw_pairs_.push_back({tick.t, imu_sum_ / imu_count_, rate});
        }
        last_heading_t_ = tick.t;
        last_heading_ = tick.our_pose->yaw;
        imu_sum_ = 0.0;
        imu_count_ = 0;
    } else if (!tick.our_pose) {
        last_heading_t_.reset();
    }

    if (tick.guard_state) guard_state_ = tick.guard_state;
    if (tick.firmware) firmware_ = tick.firmware;
    if (tick.host_protocol_version) host_protocol_version_ = tick.host_protocol_version;
}

void PerformanceMonitor::check_links(double now, std::vector<CheckResult> &firing) const {
    for (int i = 0; i < kMonitorLinkCount; ++i) {
        const auto link = static_cast<MonitorLink>(i);
        const LinkState &state = links_[static_cast<size_t>(i)];
        if (!state.expected || now - state.expected_since < config_.link_grace_s) continue;
        const LinkLimits limit = limits(link);
        const double age = link_age(state, now);
        const std::string name = lower(magic_enum::enum_name(link));
        if (age > limit.max_age_s) {
            firing.push_back({link_check(link),
                              WarningSeverity::RED,
                              fmt::format("{} silent for {:.2f} s", name, age),
                              {{"age_s", age}, {"max_age_s", limit.max_age_s}}});
            continue;
        }
        const auto rate = link_rate(state, now);
        if (rate && *rate > 0.0 && *rate < limit.min_rate_hz) {
            firing.push_back(
                {link_check(link),
                 WarningSeverity::WARN,
                 fmt::format("{} at {:.1f} Hz, below {:.1f} Hz", name, *rate, limit.min_rate_hz),
                 {{"rate_hz", *rate}, {"min_rate_hz", limit.min_rate_hz}}});
        }
    }
}

void PerformanceMonitor::check_ticks(double now, std::vector<CheckResult> &firing) const {
    Fraction slew;
    Fraction guard;
    Fraction outside;
    Fraction our;
    Fraction opponent;
    Fraction ack_low;
    Fraction age_high;
    double min_ack = 100.0;
    double max_age = 0.0;
    for (const TickRecord &record : ticks_) {
        if (record.t < now - config_.window_s) continue;
        if (!record.autonomy) continue;
        if (record.slew_bound) {
            ++slew.total;
            slew.hits += *record.slew_bound ? 1 : 0;
        }
        if (record.guard_bound) {
            ++guard.total;
            guard.hits += *record.guard_bound ? 1 : 0;
        }
        if (record.outside_field) {
            ++outside.total;
            outside.hits += *record.outside_field ? 1 : 0;
        }
        ++our.total;
        our.hits += record.our_present ? 1 : 0;
        if (record.opponent_present) {
            ++opponent.total;
            opponent.hits += *record.opponent_present ? 1 : 0;
        }
        if (record.espnow_ack_percent) {
            ++ack_low.total;
            min_ack = std::min(min_ack, *record.espnow_ack_percent);
            ack_low.hits += *record.espnow_ack_percent < config_.espnow_min_ack_percent ? 1 : 0;
        }
        if (record.command_age_ms) {
            ++age_high.total;
            max_age = std::max(max_age, *record.command_age_ms);
            age_high.hits += *record.command_age_ms > config_.espnow_max_command_age_ms ? 1 : 0;
        }
    }
    const int min_ticks = config_.min_window_ticks;
    if (slew.total >= min_ticks && slew.value() > config_.slew_tick_fraction) {
        firing.push_back({PerformanceCheck::SLEW_BINDING,
                          WarningSeverity::WARN,
                          "sent command differs from the policy command",
                          {{"fraction", slew.value()},
                           {"max_fraction", config_.slew_tick_fraction},
                           {"difference", config_.slew_difference}}});
    }
    if (guard.total >= min_ticks && guard.value() > config_.guard_tick_fraction) {
        firing.push_back({PerformanceCheck::GUARD_BINDING,
                          WarningSeverity::WARN,
                          "motor guard is scaling commands",
                          {{"fraction", guard.value()},
                           {"max_fraction", config_.guard_tick_fraction},
                           {"scale_floor", config_.guard_scale_floor}}});
    }
    if (outside.total >= min_ticks && outside.value() > config_.calibration_outside_fraction) {
        firing.push_back({PerformanceCheck::CALIBRATION,
                          WarningSeverity::WARN,
                          "our robot is outside the field box",
                          {{"fraction", outside.value()},
                           {"max_fraction", config_.calibration_outside_fraction}}});
    }
    if (our.total >= min_ticks && our.value() < config_.tracking_coverage_min) {
        firing.push_back(
            {PerformanceCheck::OUR_TRACKING,
             WarningSeverity::WARN,
             "our robot track coverage is low",
             {{"coverage", our.value()}, {"min_coverage", config_.tracking_coverage_min}}});
    }
    if (opponent.total >= min_ticks && opponent.value() < config_.tracking_coverage_min) {
        firing.push_back(
            {PerformanceCheck::OPPONENT_TRACKING,
             WarningSeverity::WARN,
             "opponent track coverage is low",
             {{"coverage", opponent.value()}, {"min_coverage", config_.tracking_coverage_min}}});
    }
    if (ack_low.total >= min_ticks && ack_low.hits > 0) {
        firing.push_back(
            {PerformanceCheck::ESPNOW_LINK,
             WarningSeverity::WARN,
             "ESP-NOW ack rate is low",
             {{"ack_percent", min_ack}, {"min_ack_percent", config_.espnow_min_ack_percent}}});
    } else if (age_high.total >= min_ticks && age_high.hits > 0) {
        firing.push_back({PerformanceCheck::ESPNOW_LINK,
                          WarningSeverity::WARN,
                          "ESP-NOW command age is high",
                          {{"command_age_ms", max_age},
                           {"max_command_age_ms", config_.espnow_max_command_age_ms}}});
    }
}

void PerformanceMonitor::check_phantoms(double now, std::vector<CheckResult> &firing) const {
    for (const auto &[name, since] : still_large_since_) {
        const double still_s = now - since;
        if (still_s < config_.phantom_min_duration_s) continue;
        firing.push_back({PerformanceCheck::PHANTOM_TRACK,
                          WarningSeverity::WARN,
                          name + " is reported with a large sigma and not moving",
                          {{"sigma_m", still_large_sigma_.at(name)},
                           {"still_s", still_s},
                           {"max_sigma_m", config_.phantom_sigma_m}}});
        return;
    }
}

void PerformanceMonitor::check_yaw(std::vector<CheckResult> &firing) const {
    if (static_cast<int>(yaw_pairs_.size()) < config_.yaw_min_pairs) return;
    std::vector<double> imu;
    std::vector<double> heading;
    for (const YawPair &pair : yaw_pairs_) {
        imu.push_back(pair.imu);
        heading.push_back(pair.heading_rate);
    }
    if (standard_deviation(imu) < config_.yaw_min_rate_std_radps) return;
    const auto correlation = pearson(imu, heading);
    const double value = correlation.value_or(0.0);
    if (value >= config_.yaw_correlation_min) return;
    firing.push_back({PerformanceCheck::YAW_RATE_DISAGREEMENT,
                      WarningSeverity::WARN,
                      "IMU yaw rate disagrees with the tracked heading",
                      {{"correlation", value},
                       {"min_correlation", config_.yaw_correlation_min},
                       {"pairs", static_cast<double>(imu.size())}}});
}

void PerformanceMonitor::check_latched(std::vector<CheckResult> &firing) const {
    if (guard_state_ && *guard_state_ == kGuardStateTripped) {
        firing.push_back({PerformanceCheck::THERMAL_TRIP,
                          WarningSeverity::RED,
                          "firmware thermal trip latched",
                          {{"guard_state", static_cast<double>(*guard_state_)}}});
    }
    if (firmware_ && host_protocol_version_ &&
        firmware_->protocol_version != *host_protocol_version_) {
        firing.push_back(
            {PerformanceCheck::FIRMWARE_PROTOCOL,
             WarningSeverity::RED,
             "firmware protocol " + std::to_string(firmware_->protocol_version) + " (build " +
                 firmware_->build + "), host speaks " + std::to_string(*host_protocol_version_),
             {{"firmware_protocol_version", static_cast<double>(firmware_->protocol_version)},
              {"host_protocol_version", static_cast<double>(*host_protocol_version_)}}});
    }
}

void PerformanceMonitor::apply(double now, const std::vector<CheckResult> &firing) {
    std::map<PerformanceCheck, const CheckResult *> by_check;
    for (const CheckResult &result : firing) {
        const auto existing = by_check.find(result.check);
        if (existing == by_check.end() || result.severity > existing->second->severity) {
            by_check[result.check] = &result;
        }
    }
    for (auto it = active_.begin(); it != active_.end();) {
        const auto firing_now = by_check.find(it->first);
        if (firing_now == by_check.end() || firing_now->second->severity != it->second.severity) {
            it->second.end_s = now;
            ended_.push_back(it->second);
            it = active_.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto &[check, result] : by_check) {
        auto [it, inserted] = active_.try_emplace(check);
        PerformanceWarning &warning = it->second;
        if (inserted) {
            warning.check = check;
            warning.start_s = now;
        }
        warning.severity = result->severity;
        warning.message = result->message;
        warning.values = result->values;
    }
}

void PerformanceMonitor::prune(double now) {
    const double boundary = now - config_.window_s;
    while (!ticks_.empty() && ticks_.front().t < boundary) ticks_.pop_front();
    while (!yaw_pairs_.empty() && yaw_pairs_.front().t < boundary) yaw_pairs_.pop_front();
    for (LinkState &state : links_) {
        while (state.samples.size() > 1 && state.samples[1].first <= boundary) {
            state.samples.pop_front();
        }
    }
}

void PerformanceMonitor::evaluate(double now) {
    for (int i = 0; i < kMonitorLinkCount; ++i) update_link(static_cast<MonitorLink>(i), now);
    prune(now);
    std::vector<CheckResult> firing;
    check_links(now, firing);
    check_ticks(now, firing);
    check_phantoms(now, firing);
    check_yaw(firing);
    check_latched(firing);
    apply(now, firing);
}

std::vector<PerformanceWarning> PerformanceMonitor::active() const {
    std::vector<PerformanceWarning> out;
    out.reserve(active_.size());
    for (const auto &[check, warning] : active_) out.push_back(warning);
    return out;
}

std::vector<LinkStatus> PerformanceMonitor::link_status(double now) const {
    std::vector<LinkStatus> out;
    for (int i = 0; i < kMonitorLinkCount; ++i) {
        const LinkState &state = links_[static_cast<size_t>(i)];
        out.push_back({static_cast<MonitorLink>(i), state.expected, state.total,
                       link_rate(state, now), state.expected ? link_age(state, now) : 0.0});
    }
    return out;
}

}  // namespace auto_battlebot
