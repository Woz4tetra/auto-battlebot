#include "control_loop/control_loop.hpp"

#include <spdlog/spdlog.h>

#include <utility>

#include "diagnostics_logger/function_timer.hpp"
#include "enums/frame_id.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {

ControlLoop::ControlLoop(std::shared_ptr<RobotFilterInterface> robot_filter,
                         std::shared_ptr<TargetSelectorInterface> target_selector,
                         std::shared_ptr<NavigationInterface> navigation,
                         std::shared_ptr<TransmitterInterface> transmitter,
                         std::shared_ptr<ClockInterface> clock, std::shared_ptr<UIState> ui_state)
    : robot_filter_(std::move(robot_filter)),
      target_selector_(std::move(target_selector)),
      navigation_(std::move(navigation)),
      transmitter_(std::move(transmitter)),
      clock_(std::move(clock)),
      ui_state_(std::move(ui_state)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("runner")) {}

bool ControlLoop::initialize() {
    if (!transmitter_->initialize()) {
        spdlog::error("Failed to initialize transmitter");
        return false;
    }
    return true;
}

void ControlLoop::pump_input() {
    command_feedback_ = transmitter_->update();
    if (transmitter_->did_init_button_press()) {
        init_button_latched_.store(true);
    }
    transmitter_connected_.store(transmitter_->is_connected());
}

bool ControlLoop::take_init_button_press() { return init_button_latched_.exchange(false); }

void ControlLoop::submit_measurement(ControlMeasurement measurement) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_measurement_ = std::move(measurement);
}

void ControlLoop::request_filter_reinit(int opponent_count) {
    filter_reinit_request_.store(opponent_count);
}

void ControlLoop::set_autonomy_enabled(bool enabled) { autonomy_enabled_.store(enabled); }

ControlOutput ControlLoop::latest_output() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_;
}

NavigationVisualization ControlLoop::last_visualization() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return visualization_;
}

std::chrono::steady_clock::time_point ControlLoop::last_cycle_time() const {
    return std::chrono::steady_clock::time_point(std::chrono::microseconds(last_cycle_us_.load()));
}

TargetSelection ControlLoop::resolve_target(const RobotDescriptionsStamped &robots,
                                            const FieldDescription &field_description) {
    if (ui_state_) {
        if (auto manual_target = ui_state_->get_manual_target()) {
            return *manual_target;
        }
    }
    if (target_selector_) {
        if (auto selected = target_selector_->get_target(robots, field_description)) {
            previous_selected_target_ = *selected;
        }
    }
    return previous_selected_target_;
}

void ControlLoop::run_cycle() {
    // Stamp on entry, not on completion. The watchdog asks "is the loop still turning", and a
    // cycle that legitimately does nothing yet (no field, so nothing to steer by) is still a
    // cycle. Stamping at the end made the watchdog fire during startup.
    last_cycle_us_.store(std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count());

    // Requests from other threads land here, so every component stays single-threaded.
    const int reinit = filter_reinit_request_.exchange(0);
    if (reinit > 0) {
        robot_filter_->initialize(reinit);
        navigation_->initialize();
        robot_descriptions_cache_.reset();
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

    std::optional<ControlMeasurement> measurement;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        measurement.swap(pending_measurement_);
    }
    if (measurement) {
        field_description_ = measurement->field_description;
    }
    // Nothing to steer by until the first measurement has defined the field.
    if (!field_description_) return;

    RobotDescriptionsStamped robots;
    {
        FunctionTimer timer(diagnostics_logger_, "robot_filter.update");
        robot_filter_->predict(clock_->now(), command_feedback_);
        if (measurement) {
            robot_filter_->correct(measurement->keypoints, measurement->field_description,
                                   measurement->camera_info, measurement->robot_blob_keypoints);
        }
        robots = robot_filter_->state();
    }

    if (measurement) {
        // Raw detection counts before the cache resolves missing critical robots, so future
        // recordings expose how often a fresh (non-stale) opponent fix was actually available.
        int their_total = 0;
        int their_live = 0;
        int our_live = 0;
        for (const auto &robot : robots.descriptions) {
            if (robot.group == Group::THEIRS) {
                ++their_total;
                if (!robot.is_stale) ++their_live;
            }
            if (robot.frame_id == FrameId::OUR_ROBOT_1 && !robot.is_stale) {
                our_live = 1;
            }
        }
        const int our_blob_no_keypoint = robot_filter_->last_our_blob_present_no_keypoint() ? 1 : 0;
        diagnostics_logger_->debug("perception",
                                   {{"their_count_total", their_total},
                                    {"their_count_live", their_live},
                                    {"our_present_live", our_live},
                                    {"our_blob_present_no_keypoint", our_blob_no_keypoint}});
    }

    // Resolve once so target selection and navigation operate on the same robot set within a
    // cycle. Substitutes the previous critical snapshot when a frame is missing OUR or THEIRS.
    auto cached_robots = robot_descriptions_cache_.resolve(robots);
    diagnostics_logger_->debug(
        "navigation", {{"using_previous_robots", static_cast<int>(cached_robots.using_previous)}});

    TargetSelection resolved_target = resolve_target(cached_robots.robots, *field_description_);
    VelocityCommand command =
        navigation_->update(cached_robots.robots, *field_description_, resolved_target);
    transmitter_->send(command);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        output_.valid = true;
        output_.robots = robots;
        output_.cached_robots = cached_robots.robots;
        output_.using_previous = cached_robots.using_previous;
        output_.target = resolved_target;
        output_.command = command;
        output_.our_blob_present_no_keypoint = robot_filter_->last_our_blob_present_no_keypoint();
        visualization_ = navigation_->get_last_visualization();
    }
}

}  // namespace auto_battlebot
