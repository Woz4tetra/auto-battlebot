#include "health/mcap_grader.hpp"

#include <spdlog/fmt/fmt.h>

#include <cmath>
#include <magic_enum.hpp>
#include <mcap/reader.hpp>
#include <nlohmann/json.hpp>
#include <numbers>
#include <set>
#include <string_view>

namespace auto_battlebot {
namespace {

using nlohmann::json;

constexpr std::string_view kFrameMeta = "/camera/frame_meta";
constexpr std::string_view kVideo = "/camera/video";
constexpr std::string_view kDetections = "/keypoint_detections";
constexpr std::string_view kTracks = "/status/tracks";
constexpr std::string_view kSystem = "/status/system";
constexpr std::string_view kConfig = "/status/config";
constexpr std::string_view kTelemetry = "/robot/espnow_telemetry";
constexpr std::string_view kEspNowDiagnostics = "/diagnostics/espnow_transmitter";
constexpr std::string_view kOpenTxDiagnostics = "/diagnostics/opentx_transmitter";
constexpr std::string_view kPolicyDiagnostics = "/diagnostics/rl_policy_nav";
constexpr std::string_view kKalmanDiagnostics = "/diagnostics/kalman_motion_estimator";
constexpr std::string_view kOpponentPrefix = "opponent_";
constexpr double kFreshSampleS = 0.25;
constexpr double kShutdownTailS = 1.0;

const std::set<std::string, std::less<>> kCrsfSubsections = {"link_statistics", "battery",
                                                             "attitude", "flight_mode"};

std::string lower(std::string text) {
    for (char &c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return text;
}

std::optional<double> number(const json &object, const char *key) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_number()) return std::nullopt;
    return it->get<double>();
}

bool boolean(const json &object, const char *key, bool fallback) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_boolean()) return fallback;
    return it->get<bool>();
}

struct Sample {
    double t = 0.0;
    double value = 0.0;
};

std::optional<double> fresh(const std::optional<Sample> &sample, double now) {
    if (!sample || now - sample->t > kFreshSampleS) return std::nullopt;
    return sample->value;
}

class Replay {
   public:
    Replay(const PerformanceMonitorConfiguration &config, McapGrade &grade)
        : monitor_(config), config_(config), grade_(grade) {}

    void expect_from_config(const std::optional<std::string> &transmitter,
                            const std::set<std::string, std::less<>> &topics) {
        const auto has = [&topics](std::string_view topic) { return topics.count(topic) != 0; };
        has_frame_meta_ = has(kFrameMeta);
        has_system_ = has(kSystem);
        initialized_ = !has_system_ ? false : initialized_;
        const bool espnow = transmitter == "EspNowTransmitter";
        const bool opentx = transmitter == "OpenTxTransmitter";
        recorded_[MonitorLink::CAMERA_FRAMES] = has_frame_meta_ || has(kVideo);
        recorded_[MonitorLink::PERCEPTION] = has(kDetections);
        recorded_[MonitorLink::FILTER] = has(kTracks);
        recorded_[MonitorLink::ESPNOW] = espnow && has(kTelemetry);
        recorded_[MonitorLink::IMU] = espnow && has(kTelemetry);
        recorded_[MonitorLink::CRSF_TELEMETRY] = opentx && has(kOpenTxDiagnostics);
        recorded_[MonitorLink::TRANSMIT] =
            has(kSystem) && (recorded_[MonitorLink::ESPNOW] || has(kOpenTxDiagnostics));
        if (!has(kSystem)) {
            grade_.notes.push_back(
                "no /status/system: autonomy state unknown, transmit link not judged");
        }
        for (const auto &[link, present] : recorded_) {
            if (present) continue;
            if (link == MonitorLink::TRANSMIT && !has(kSystem)) continue;
            const bool configured =
                link == MonitorLink::CAMERA_FRAMES || link == MonitorLink::PERCEPTION ||
                link == MonitorLink::FILTER ||
                ((link == MonitorLink::ESPNOW || link == MonitorLink::IMU) && espnow) ||
                (link == MonitorLink::CRSF_TELEMETRY && opentx) ||
                (link == MonitorLink::TRANSMIT && (espnow || opentx));
            if (!configured) continue;
            grade_.notes.push_back(std::string(magic_enum::enum_name(link)) +
                                   " has no source topic in this recording: link not judged");
        }
    }

    void handle(std::string_view topic, double t, std::string_view payload) {
        advance_to(t);
        if (topic == kFrameMeta || (topic == kVideo && !has_frame_meta_)) {
            monitor_.add_link_events(MonitorLink::CAMERA_FRAMES);
        } else if (topic == kDetections) {
            if (!has_system_) initialized_ = true;
            monitor_.add_link_events(MonitorLink::PERCEPTION);
        } else if (topic == kSystem) {
            handle_system(parse(payload));
        } else if (topic == kTracks) {
            monitor_.add_link_events(MonitorLink::FILTER);
            handle_tracks(t, parse(payload));
        } else if (topic == kTelemetry) {
            handle_telemetry(t, parse(payload));
        } else if (topic == kEspNowDiagnostics) {
            for_each_fresh_subsection(parse(payload), [&](const std::string &name, const json &s) {
                if (name != "link") return;
                if (const auto ack = number(s.value("values", json::object()), "ack_percent")) {
                    ack_ = Sample{t, *ack};
                }
            });
        } else if (topic == kOpenTxDiagnostics) {
            for_each_fresh_subsection(parse(payload), [&](const std::string &name, const json &s) {
                if (kCrsfSubsections.count(name) != 0) {
                    monitor_.add_link_events(MonitorLink::CRSF_TELEMETRY);
                } else if (name == "send") {
                    monitor_.add_link_events(MonitorLink::TRANSMIT);
                } else if (name == "switch_states") {
                    const json values = s.value("values", json::object());
                    if (const auto on = number(values, "autonomy_enabled")) {
                        switch_on_ = *on != 0.0;
                        saw_switch_ = true;
                    }
                }
            });
        } else if (topic == kPolicyDiagnostics) {
            for_each_fresh_subsection(parse(payload), [&](const std::string &name, const json &s) {
                if (name != "policy") return;
                if (const auto linear = number(s.value("values", json::object()), "linear")) {
                    policy_linear_ = Sample{t, *linear};
                    saw_policy_ = true;
                }
            });
        } else if (topic == kKalmanDiagnostics) {
            for_each_fresh_subsection(parse(payload), [&](const std::string &name, const json &s) {
                if (!name.starts_with(kOpponentPrefix)) return;
                const auto sigma =
                    number(s.value("values", json::object()), "position_sigma_major_axis_m");
                if (sigma) sigma_by_track_[lower(name.substr(kOpponentPrefix.size()))] = *sigma;
            });
        }
        set_expectations(t);
    }

    void finish(double t_end) {
        advance_to(t_end);
        monitor_.evaluate(t_end);
        for (const LinkStatus &status : monitor_.link_status(t_end)) {
            grade_.link_totals[lower(std::string(magic_enum::enum_name(status.link)))] =
                status.total;
        }
        std::vector<PerformanceWarning> warnings = monitor_.ended();
        for (PerformanceWarning warning : monitor_.active()) {
            warning.end_s = t_end;
            warnings.push_back(std::move(warning));
        }
        int shutdown_tail = 0;
        for (PerformanceWarning &warning : warnings) {
            if (warning.start_s > t_end - kShutdownTailS) {
                ++shutdown_tail;
                continue;
            }
            grade_.warnings.push_back(std::move(warning));
        }
        if (shutdown_tail > 0) {
            grade_.notes.push_back(
                fmt::format("{} warning(s) starting in the last {:.0f} s dropped as shutdown",
                            shutdown_tail, kShutdownTailS));
        }
        if (!saw_tracks_)
            grade_.notes.push_back(
                "no /status/tracks: calibration, tracking and "
                "yaw-rate checks did not run");
        if (!saw_policy_)
            grade_.notes.push_back(
                "no rl_policy_nav policy samples: slew binding "
                "did not run");
        if (sigma_by_track_.empty())
            grade_.notes.push_back(
                "no kalman_motion_estimator opponent "
                "sigma: phantom track did not run");
        if (recorded_[MonitorLink::CRSF_TELEMETRY] && !saw_switch_) {
            grade_.notes.push_back("no autonomy switch state: CRSF telemetry link not judged");
        }
        if (!has_system_ && (recorded_[MonitorLink::ESPNOW] || recorded_[MonitorLink::IMU])) {
            grade_.notes.push_back("no /status/system: ESP-NOW and IMU links not judged");
        }
        grade_.notes.push_back("relay publish is not recorded: its link did not run");
    }

   private:
    static json parse(std::string_view payload) {
        return json::parse(payload.begin(), payload.end(), nullptr, false);
    }

    template <typename F>
    static void for_each_fresh_subsection(const json &message, F &&f) {
        if (!message.is_object()) return;
        for (const auto &[name, subsection] : message.items()) {
            if (!subsection.is_object() || boolean(subsection, "held", false)) continue;
            f(name, subsection);
        }
    }

    void advance_to(double t) {
        if (!next_eval_) next_eval_ = t;
        while (*next_eval_ <= t) {
            monitor_.evaluate(*next_eval_);
            *next_eval_ += config_.evaluate_period_s;
        }
    }

    bool autonomy_driving() const {
        return initialized_ && autonomy_enabled_ && (!saw_switch_ || switch_on_);
    }

    void expect(MonitorLink link, bool when, double t) {
        monitor_.set_link_expected(link, recorded_[link] && when, t);
    }

    void set_expectations(double t) {
        expect(MonitorLink::CAMERA_FRAMES, true, t);
        expect(MonitorLink::PERCEPTION, initialized_, t);
        expect(MonitorLink::FILTER, initialized_, t);
        const bool host_driving = has_system_ && initialized_ && autonomy_enabled_;
        expect(MonitorLink::TRANSMIT, host_driving, t);
        expect(MonitorLink::ESPNOW, host_driving, t);
        expect(MonitorLink::IMU, host_driving, t);
        expect(MonitorLink::CRSF_TELEMETRY, saw_switch_ && switch_on_, t);
    }

    void handle_system(const json &message) {
        if (!message.is_object()) return;
        initialized_ = boolean(message, "initialized", initialized_);
        autonomy_enabled_ = boolean(message, "autonomy_enabled", autonomy_enabled_);
        const auto it = message.find("autonomy_switch_on");
        if (it != message.end() && it->is_boolean()) {
            switch_on_ = it->get<bool>();
            saw_switch_ = true;
        }
    }

    void handle_telemetry(double t, const json &message) {
        if (!message.is_object()) return;
        monitor_.add_link_events(MonitorLink::ESPNOW);
        const bool upside_down = boolean(message, "is_upside_down", false);
        if (const auto yaw_rate = number(message, "yaw_rate")) {
            monitor_.add_link_events(MonitorLink::IMU);
            const double cw = *yaw_rate * std::numbers::pi / 180.0;
            imu_sum_ += upside_down ? cw : -cw;
            ++imu_count_;
        }
        if (const auto seq = number(message, "espnow_seq")) {
            const auto value = static_cast<uint64_t>(*seq);
            if (last_espnow_seq_ && value != *last_espnow_seq_) {
                monitor_.add_link_events(MonitorLink::TRANSMIT);
            }
            last_espnow_seq_ = value;
        }
        if (const auto scale = number(message, "guard_scale")) guard_scale_ = Sample{t, *scale};
        if (const auto state = number(message, "guard_state")) guard_state_ = Sample{t, *state};
        if (const auto age = number(message, "espnow_age_ms")) command_age_ = Sample{t, *age};
        if (boolean(message, "espnow_active", false)) {
            if (const auto a = number(message, "a_percent")) {
                const double linear = (upside_down ? *a : -*a) / 100.0;
                sent_linear_ = Sample{t, linear};
            }
        }
    }

    void handle_tracks(double t, const json &message) {
        if (!message.is_object()) return;
        saw_tracks_ = true;
        MonitorTick tick;
        tick.t = t;
        tick.autonomy_driving = autonomy_driving();
        const auto field_x = number(message, "field_x");
        const auto field_y = number(message, "field_y");
        if (field_x && field_y && *field_x > 0.0 && *field_y > 0.0) {
            tick.field = FieldBox{.half_x = 0.5 * *field_x, .half_y = 0.5 * *field_y};
        }
        tick.opponent_expected = true;
        std::map<std::string, Pose2D> positions;
        for (const json &robot : message.value("robots", json::array())) {
            const std::string id = robot.value("id", std::string());
            const Pose2D pose{robot.value("x", 0.0), robot.value("y", 0.0),
                              robot.value("yaw", 0.0)};
            positions[id] = pose;
            if (robot.value("ours", false)) {
                tick.our_pose = pose;
                tick.our_heading_measured = true;
            } else {
                tick.opponent_present = true;
            }
            const auto sigma = sigma_by_track_.find(id);
            if (sigma == sigma_by_track_.end()) continue;
            double speed = 0.0;
            const auto previous = last_positions_.find(id);
            if (previous != last_positions_.end() && t > last_tracks_t_) {
                speed = std::hypot(pose.x - previous->second.x, pose.y - previous->second.y) /
                        (t - last_tracks_t_);
            }
            tick.tracks.push_back({.name = id, .sigma_m = sigma->second, .speed_mps = speed});
        }
        last_positions_ = std::move(positions);
        last_tracks_t_ = t;
        if (imu_count_ > 0) {
            tick.imu_yaw_rate_ccw_radps = imu_sum_ / imu_count_;
            imu_sum_ = 0.0;
            imu_count_ = 0;
        }
        tick.policy_linear = fresh(policy_linear_, t);
        tick.sent_linear = fresh(sent_linear_, t);
        tick.guard_scale = fresh(guard_scale_, t);
        if (const auto state = fresh(guard_state_, t)) tick.guard_state = static_cast<int>(*state);
        tick.command_age_ms = fresh(command_age_, t);
        if (ack_ && t - ack_->t < 2.0) tick.espnow_ack_percent = ack_->value;
        monitor_.record_tick(tick);
    }

    PerformanceMonitor monitor_;
    PerformanceMonitorConfiguration config_;
    McapGrade &grade_;
    std::optional<double> next_eval_;
    std::map<MonitorLink, bool> recorded_;
    bool has_frame_meta_ = false;
    bool has_system_ = false;
    bool initialized_ = true;
    bool autonomy_enabled_ = true;
    bool switch_on_ = false;
    bool saw_switch_ = false;
    bool saw_tracks_ = false;
    bool saw_policy_ = false;
    std::map<std::string, double> sigma_by_track_;
    std::map<std::string, Pose2D> last_positions_;
    double last_tracks_t_ = 0.0;
    double imu_sum_ = 0.0;
    int imu_count_ = 0;
    std::optional<uint64_t> last_espnow_seq_;
    std::optional<Sample> guard_scale_;
    std::optional<Sample> guard_state_;
    std::optional<Sample> command_age_;
    std::optional<Sample> sent_linear_;
    std::optional<Sample> policy_linear_;
    std::optional<Sample> ack_;
};

std::string_view payload_of(const mcap::MessageView &view) {
    return {reinterpret_cast<const char *>(view.message.data), view.message.dataSize};
}

std::optional<std::string> transmitter_from_config(mcap::McapReader &reader) {
    mcap::ReadMessageOptions options;
    options.topicFilter = [](std::string_view topic) { return topic == kConfig; };
    for (const auto &view : reader.readMessages([](const mcap::Status &) {}, options)) {
        const json message = json::parse(payload_of(view), nullptr, false);
        if (!message.is_object()) continue;
        for (const json &field : message.value("fields", json::array())) {
            if (field.value("section", "") == "transmitter" && field.value("key", "") == "type") {
                std::string value = field.value("value", "");
                if (value.size() >= 2 && (value.front() == '"' || value.front() == '\'')) {
                    value = value.substr(1, value.size() - 2);
                }
                return value;
            }
        }
    }
    return std::nullopt;
}

}  // namespace

bool McapGrade::red() const {
    for (const auto &warning : warnings) {
        if (warning.severity == WarningSeverity::RED) return true;
    }
    return false;
}

McapGrade grade_mcap(const std::filesystem::path &path,
                     const PerformanceMonitorConfiguration &config) {
    McapGrade grade;
    grade.path = path;
    mcap::McapReader reader;
    const mcap::Status opened = reader.open(path.string());
    if (!opened.ok())
        throw std::runtime_error("cannot open " + path.string() + ": " + opened.message);
    const mcap::Status summary = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!summary.ok()) grade.notes.push_back("summary unreadable: " + summary.message);

    std::set<std::string, std::less<>> topics;
    for (const auto &[id, channel] : reader.channels()) {
        (void)id;
        topics.insert(channel->topic);
    }
    grade.transmitter_type = transmitter_from_config(reader);
    if (!grade.transmitter_type) {
        if (topics.count(kTelemetry) != 0) {
            grade.transmitter_type = "EspNowTransmitter";
        } else if (topics.count(kOpenTxDiagnostics) != 0) {
            grade.transmitter_type = "OpenTxTransmitter";
        }
        grade.notes.push_back("no /status/config: transmitter type inferred from topics");
    }

    Replay replay(config, grade);
    replay.expect_from_config(grade.transmitter_type, topics);
    std::optional<uint64_t> first_ns;
    uint64_t last_ns = 0;
    mcap::ReadMessageOptions options;
    options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
    const auto on_problem = [&grade](const mcap::Status &status) {
        if (grade.notes.size() < 20) grade.notes.push_back("read problem: " + status.message);
    };
    for (const auto &view : reader.readMessages(on_problem, options)) {
        const uint64_t log_ns = view.message.logTime;
        if (!first_ns) first_ns = log_ns;
        last_ns = std::max(last_ns, log_ns);
        const double t = static_cast<double>(log_ns - *first_ns) * 1e-9;
        ++grade.topic_counts[view.channel->topic];
        replay.handle(view.channel->topic, t, payload_of(view));
    }
    grade.duration_s = first_ns ? static_cast<double>(last_ns - *first_ns) * 1e-9 : 0.0;
    replay.finish(grade.duration_s);
    return grade;
}

std::string format_grade(const McapGrade &grade) {
    std::string out = fmt::format("{}\n  duration {:.1f} s, transmitter {}\n", grade.path.string(),
                                  grade.duration_s, grade.transmitter_type.value_or("unknown"));
    if (grade.warnings.empty()) out += "  no warnings\n";
    for (const PerformanceWarning &warning : grade.warnings) {
        out += fmt::format("  {:<4} {:<22} {:7.1f}-{:7.1f} s  {}",
                           warning.severity == WarningSeverity::RED ? "RED" : "WARN",
                           check_name(warning.check), warning.start_s,
                           warning.end_s.value_or(grade.duration_s), warning.message);
        for (const auto &[key, value] : warning.values)
            out += fmt::format(" {}={:.3g}", key, value);
        out += "\n";
    }
    out += "  link totals:";
    for (const auto &[link, total] : grade.link_totals) out += fmt::format(" {}={}", link, total);
    out += "\n";
    for (const std::string &note : grade.notes) out += "  note: " + note + "\n";
    out += grade.red() ? "  verdict: RED\n" : "  verdict: OK\n";
    return out;
}

}  // namespace auto_battlebot
