#include "control_loop/control_loop.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <magic_enum.hpp>
#include <utility>

#include "diagnostics_logger/function_timer.hpp"
#include "enums/frame_id.hpp"
#include "time_utils.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

ControlLoop::ControlLoop(std::shared_ptr<RobotFilterInterface> robot_filter,
                         std::shared_ptr<TargetSelectorInterface> target_selector,
                         std::shared_ptr<NavigationInterface> navigation,
                         std::shared_ptr<CommandShaperInterface> command_shaper,
                         std::shared_ptr<TransmitterInterface> transmitter,
                         std::shared_ptr<ClockInterface> clock,
                         std::shared_ptr<HazardAssembler> hazard_assembler)
    : robot_filter_(std::move(robot_filter)),
      target_selector_(std::move(target_selector)),
      navigation_(std::move(navigation)),
      command_shaper_(std::move(command_shaper)),
      transmitter_(std::move(transmitter)),
      clock_(std::move(clock)),
      hazard_assembler_(std::move(hazard_assembler)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("runner")) {}

bool ControlLoop::initialize() {
    if (!transmitter_->initialize()) {
        spdlog::error("Failed to initialize transmitter");
        return false;
    }
    return true;
}

void ControlLoop::set_performance_monitor(std::shared_ptr<PerformanceMonitor> monitor) {
    monitor_ = std::move(monitor);
    monitor_logger_ = monitor_ ? DiagnosticsLogger::get_logger("performance_monitor") : nullptr;
}

void ControlLoop::note_camera_frame() {
    camera_frame_seen_.store(true);
    if (monitor_) monitor_->add_link_events(MonitorLink::CAMERA_FRAMES);
}

void ControlLoop::pump_input() {
    command_feedback_ = transmitter_->update();
    if (transmitter_->did_init_button_press()) {
        init_button_latched_.store(true);
    }
    transmitter_status_.store(transmitter_->get_status());
}

bool ControlLoop::take_init_button_press() { return init_button_latched_.exchange(false); }

void ControlLoop::submit_measurement(ControlMeasurement measurement) {
    if (monitor_) monitor_->add_link_events(MonitorLink::PERCEPTION);
    std::lock_guard<std::mutex> lock(mutex_);
    pending_measurement_ = std::move(measurement);
}

void ControlLoop::request_filter_reinit(int opponent_count) {
    opponent_count_.store(opponent_count);
    filter_reinit_request_.store(opponent_count);
}

void ControlLoop::set_autonomy_enabled(bool enabled) { autonomy_enabled_.store(enabled); }

void ControlLoop::set_behavior_mode(BehaviorMode mode) { behavior_mode_.store(mode); }

void ControlLoop::set_target_override(std::optional<Pose2D> target) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_override_ = target;
}

ControlOutput ControlLoop::latest_output() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_;
}

NavigationVisualization ControlLoop::last_visualization() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return visualization_;
}

std::chrono::steady_clock::time_point ControlLoop::last_cycle_wall_time() const {
    return std::chrono::steady_clock::time_point(std::chrono::microseconds(last_cycle_us_.load()));
}

TargetSelection ControlLoop::resolve_target(const RobotDescriptionsStamped &robots,
                                            const FieldDescription &field_description,
                                            BehaviorMode mode,
                                            const std::optional<Pose2D> &target_override) {
    if (target_override) {
        return TargetSelection{.pose = *target_override, .label = Label::EMPTY, .mode = mode};
    }
    if (target_selector_) {
        if (auto selected = target_selector_->get_target(robots, field_description, mode)) {
            previous_selected_target_ = *selected;
        }
    }
    previous_selected_target_.mode = mode;
    return previous_selected_target_;
}

void ControlLoop::apply_requests_from_other_threads() {
    const int reinit = filter_reinit_request_.exchange(0);
    if (reinit > 0) {
        robot_filter_->initialize(reinit);
        navigation_->initialize();
        previous_selected_target_ = TargetSelection{};
    }
    const int autonomy = autonomy_enabled_.load() ? 1 : 0;
    if (autonomy != autonomy_applied_.load()) {
        autonomy_applied_.store(autonomy);
        if (autonomy == 1) {
            transmitter_->enable();
        } else {
            transmitter_->disable();
        }
    }
}

void ControlLoop::run_cycle_at(double now) {
    clock_->step_to(now);
    run_cycle();
}

void ControlLoop::run_cycle() {
    last_cycle_us_.store(std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count());

    apply_requests_from_other_threads();

    std::optional<ControlMeasurement> measurement;
    std::optional<Pose2D> target_override;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        measurement.swap(pending_measurement_);
        target_override = target_override_;
    }
    if (measurement) {
        field_description_ = measurement->field_description;
    }
    if (!field_description_) {
        diagnostics_logger_->debug("navigation", "waiting for the field; no commands sent");
        if (monitor_ && camera_frame_seen_.load()) {
            monitor_cycle(clock_->now(), nullptr, std::nullopt, false);
        }
        return;
    }

    RobotDescriptionsStamped robots;
    {
        FunctionTimer timer(diagnostics_logger_, "robot_filter.update");
        if (measurement) {
            robot_filter_->correct(measurement->keypoints, measurement->field_description,
                                   measurement->camera_info);
        }
        robot_filter_->predict(clock_->now(), command_feedback_);
        robots = robot_filter_->state();
    }

    const auto log_raw_detection_counts = [&]() {
        int their_live = 0;
        int our_live = 0;
        for (const auto &robot : robots.descriptions) {
            if (robot.group == Group::THEIRS) ++their_live;
            if (robot.frame_id == FrameId::OUR_ROBOT_1) our_live = 1;
        }
        diagnostics_logger_->debug(
            "perception", {{"their_count_live", their_live}, {"our_present_live", our_live}});
    };
    if (measurement) log_raw_detection_counts();

    const BehaviorMode behavior_mode = behavior_mode_.load();
    diagnostics_logger_->debug(
        "navigation", {{"behavior_mode", std::string(magic_enum::enum_name(behavior_mode))}});

    if (hazard_assembler_) {
        hazard_assembler_->assemble(robots, *field_description_);
    }

    TargetSelection resolved_target =
        resolve_target(robots, *field_description_, behavior_mode, target_override);
    VelocityCommand command = navigation_->update(robots, *field_description_, resolved_target);
    const StickCommand shaped = command_shaper_->shape(command);
    transmitter_->send(shaped);
    monitor_cycle(clock_->now(), &robots, shaped, measurement.has_value());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        output_.valid = true;
        output_.robots = robots;
        output_.target = resolved_target;
        output_.command = command;
        output_.command_feedback = command_feedback_;
        output_.behavior_mode = behavior_mode;
        output_.manual_target = target_override.has_value();
        output_.hazards = field_description_->hazards;
        visualization_ = navigation_->get_last_visualization();
    }
}

namespace {

std::string frame_name(FrameId frame_id) {
    std::string name(magic_enum::enum_name(frame_id));
    for (char &c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return name;
}

DiagnosticLevel warning_level(WarningSeverity severity) {
    return severity == WarningSeverity::RED ? DiagnosticLevel::ERROR : DiagnosticLevel::WARN;
}

}  // namespace

void ControlLoop::monitor_cycle(double now, const RobotDescriptionsStamped *robots,
                                const std::optional<StickCommand> &shaped, bool measured) {
    if (!monitor_) return;
    const TransmitterLinkReport report = transmitter_->link_report();
    const TransmitterStatus status = transmitter_->get_status();
    const bool autonomy_enabled = autonomy_enabled_.load();
    const bool switch_on = !status.has_trainer_autonomy_switch || status.autonomy_switch_on;
    const bool field_ready = robots != nullptr;

    monitor_->set_link_expected(MonitorLink::CAMERA_FRAMES, true, now);
    monitor_->set_link_expected(MonitorLink::PERCEPTION, field_ready, now);
    monitor_->set_link_expected(MonitorLink::FILTER, field_ready, now);
    monitor_->set_link_expected(MonitorLink::RELAY_PUBLISH,
                                monitor_->has_link_source(MonitorLink::RELAY_PUBLISH), now);
    if (field_ready) monitor_->add_link_events(MonitorLink::FILTER);
    for (MonitorLink link : report.links) {
        switch (link) {
            case MonitorLink::TRANSMIT:
                monitor_->set_link_total(link, report.commands_sent);
                monitor_->set_link_expected(link, field_ready && autonomy_enabled, now);
                break;
            case MonitorLink::CRSF_TELEMETRY:
                monitor_->set_link_total(link, report.telemetry_frames);
                monitor_->set_link_expected(link, status.autonomy_switch_on, now);
                break;
            case MonitorLink::ESPNOW:
                monitor_->set_link_total(link, report.telemetry_frames);
                monitor_->set_link_expected(link, field_ready && autonomy_enabled, now);
                break;
            case MonitorLink::IMU:
                monitor_->set_link_total(link, report.imu_samples);
                monitor_->set_link_expected(link, field_ready && autonomy_enabled, now);
                break;
            default:
                break;
        }
    }

    MonitorTick tick;
    tick.t = now;
    tick.autonomy_driving = field_ready && autonomy_enabled && switch_on;
    const bool sent_this_cycle = report.commands_sent != last_commands_sent_;
    last_commands_sent_ = report.commands_sent;
    if (shaped && sent_this_cycle && report.last_sent) {
        tick.policy_linear = shaped->forward;
        tick.sent_linear = report.last_sent->forward;
    }
    tick.guard_scale = report.guard_scale;
    tick.guard_state = report.guard_state;
    tick.imu_yaw_rate_ccw_radps = report.imu_yaw_rate_ccw_radps;
    tick.espnow_ack_percent = report.ack_percent;
    tick.command_age_ms = report.command_age_ms;
    tick.firmware = report.firmware;
    tick.host_protocol_version = report.host_protocol_version;
    if (field_ready && field_description_) {
        tick.field = FieldBox{.half_x = 0.5 * field_description_->size.size.x,
                              .half_y = 0.5 * field_description_->size.size.y};
        tick.opponent_expected = opponent_count_.load() > 0;
        for (const auto &robot : robots->descriptions) {
            if (robot.frame_id == FrameId::OUR_ROBOT_1) {
                tick.our_pose = pose_to_pose2d(robot.pose);
                tick.our_heading_measured = measured;
            }
            if (robot.group == Group::THEIRS) tick.opponent_present = true;
            if (robot.uncertainty.position_sigma_major_axis_m) {
                tick.tracks.push_back(
                    {.name = frame_name(robot.frame_id),
                     .sigma_m = *robot.uncertainty.position_sigma_major_axis_m,
                     .speed_mps = std::hypot(robot.velocity.vx_mps, robot.velocity.vy_mps)});
            }
        }
    }
    monitor_->record_tick(tick);

    const PerformanceMonitorConfiguration &config = monitor_->config();
    if (!next_evaluate_s_ || now >= *next_evaluate_s_) {
        monitor_->evaluate(now);
        next_evaluate_s_ = now + config.evaluate_period_s;
    }
    if (!next_publish_s_ || now >= *next_publish_s_) {
        publish_monitor(now);
        next_publish_s_ = now + config.publish_period_s;
    }
}

void ControlLoop::publish_monitor(double now) {
    std::vector<PerformanceCheck> still_active;
    for (const PerformanceWarning &warning : monitor_->active()) {
        DiagnosticsData data;
        for (const auto &[key, value] : warning.values) data[key] = value;
        data["start_s"] = warning.start_s;
        data["duration_s"] = now - warning.start_s;
        monitor_logger_->log(warning_level(warning.severity), check_name(warning.check), data,
                             warning.message);
        still_active.push_back(warning.check);
    }
    for (PerformanceCheck check : published_warnings_) {
        if (std::find(still_active.begin(), still_active.end(), check) != still_active.end()) {
            continue;
        }
        DiagnosticsData data;
        for (auto it = monitor_->ended().rbegin(); it != monitor_->ended().rend(); ++it) {
            if (it->check != check) continue;
            data["start_s"] = it->start_s;
            data["end_s"] = it->end_s.value_or(now);
            break;
        }
        monitor_logger_->info(check_name(check), data, "cleared");
    }
    published_warnings_ = std::move(still_active);

    for (const LinkStatus &link : monitor_->link_status(now)) {
        if (!link.expected) continue;
        const PerformanceCheck check = link_check(link.link);
        if (std::find(published_warnings_.begin(), published_warnings_.end(), check) !=
            published_warnings_.end()) {
            continue;
        }
        DiagnosticsData data{{"total", static_cast<int>(link.total)}, {"age_s", link.age_s}};
        if (link.rate_hz) data["rate_hz"] = *link.rate_hz;
        monitor_logger_->info(check_name(check), data);
    }
}

}  // namespace auto_battlebot
