#include <miniros/ros.h>

#include <CLI/CLI.hpp>
#include <atomic>
#include <csignal>
#include <diagnostic_msgs/DiagnosticArray.hxx>
#include <memory>
#include <thread>
#include <vector>

#include "config/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/ros_diagnostics_backend.hpp"
#include "diagnostics_logger/ui_diagnostics_backend.hpp"
#include "runner.hpp"
#include "ui/ui_runner.hpp"
#include "ui/ui_state.hpp"

namespace {
std::atomic<auto_battlebot::UIState *> g_ui_state_for_signal{nullptr};

void signal_quit(int) {
    auto *s = g_ui_state_for_signal.load(std::memory_order_relaxed);
    if (s) s->quit_requested.store(true);
}
}  // namespace

int main(int argc, char **argv) {
    using namespace auto_battlebot;

    CLI::App app{"Auto BattleBot - Autonomous robot control system"};
    std::string config_path = "";
    app.add_option("-c,--config", config_path, "Path to configuration directory")
        ->check(CLI::ExistingDirectory);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }

    ClassConfiguration class_config = load_classes_from_config(config_path);
    std::vector<RobotConfig> robot_configs = load_robots_from_config(config_path);

    std::map<std::string, std::string> remappings;
    miniros::init(remappings, "auto_battlebot");
    miniros::NodeHandle nh;

    std::shared_ptr<UIState> ui_state;
    std::vector<std::shared_ptr<DiagnosticsBackend>> backends;

    if (class_config.ui && class_config.ui->enable) {
        ui_state = std::make_shared<UIState>();
        backends.push_back(std::make_shared<UIDiagnosticsBackend>(ui_state));
    }

    if (class_config.publisher->type == "RosPublisher") {
        auto ros_diag_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100));
        backends.push_back(std::make_shared<RosDiagnosticsBackend>(ros_diag_publisher));
    }

    DiagnosticsLogger::initialize(backends);

    std::shared_ptr<PublisherInterface> publisher = make_publisher(nh, *class_config.publisher);

    auto camera = make_rgbd_camera(*class_config.camera);
    auto field_model = make_field_model(*class_config.field_model);
    auto floor_model = make_field_model(*class_config.floor_model);
    auto field_filter = make_field_filter(*class_config.field_filter);
    auto keypoint_model = make_keypoint_model(*class_config.keypoint_model);
    auto robot_filter = make_robot_filter(*class_config.robot_filter);
    auto navigation = make_navigation(*class_config.navigation);
    auto transmitter = make_transmitter(*class_config.transmitter);

    Runner runner(class_config.runner, robot_configs, camera, field_model, floor_model,
                  field_filter, keypoint_model, robot_filter, navigation, transmitter, publisher,
                  ui_state);

    runner.initialize();

    std::thread ui_thread;
    if (class_config.ui && class_config.ui->enable && ui_state) {
        g_ui_state_for_signal.store(ui_state.get(), std::memory_order_relaxed);
        std::signal(SIGINT, signal_quit);
        std::signal(SIGTERM, signal_quit);
        ui_state->set_window_size(class_config.ui->width, class_config.ui->height);
        ui_state->set_fullscreen(class_config.ui->fullscreen);
        ui_state->set_rate_avg_window(class_config.ui->rate_avg_window);
        ui_state->set_max_loop_rate(class_config.runner.max_loop_rate);
        ui_state->set_rate_fail_threshold(class_config.ui->rate_fail_threshold);
        ui_state->set_rate_fail_duration_sec(class_config.ui->rate_fail_duration_sec);
        ui_thread = std::thread(run_ui_thread, ui_state);
    }

    int result = runner.run();

    if (ui_thread.joinable()) {
        ui_thread.join();
    }

    g_ui_state_for_signal.store(nullptr, std::memory_order_relaxed);
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);
    return result;
}
