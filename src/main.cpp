#include <miniros/ros.h>
#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <csignal>
#include <diagnostic_msgs/DiagnosticArray.hxx>
#include <filesystem>
#include <memory>
#include <vector>

#include "config/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/ros_diagnostics_backend.hpp"
#include "logging/logging.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "quittable.hpp"
#include "runner.hpp"
#include "ui/system_actions.hpp"
#include "ui/ui_manager.hpp"

namespace {
constexpr std::size_t kMaxQuittables = 8;
auto_battlebot::Quittable* g_quittables[kMaxQuittables] = {};
std::size_t g_quittables_count = 0;

void signal_quit(int) {
    for (std::size_t i = 0; i < g_quittables_count; ++i) g_quittables[i]->request_quit();
}
}  // namespace

int main(int argc, char** argv) {
    using namespace auto_battlebot;

    CLI::App app{"Auto BattleBot - Autonomous robot control system"};
    std::string config_path_string = "";
    app.add_option("-c,--config", config_path_string,
                   "Path to config profile (e.g. config/playback.toml or config/playback)");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    std::filesystem::path config_path = normalize_config_path(config_path_string);
    ClassConfiguration class_config = load_classes_from_config(config_path);

    auto mcap_recorder = make_mcap_recorder(class_config.mcap_recorder, config_path);
    setup_logging(mcap_recorder);
    std::map<std::string, std::string> remappings;
    miniros::init(remappings, "auto_battlebot");
    miniros::NodeHandle nh;
    setup_rosout_publisher(nh);

    std::unique_ptr<UIManager> ui_manager;
    std::vector<std::shared_ptr<DiagnosticsBackend>> backends;

    if (class_config.ui && class_config.ui->enable) {
        ui_manager =
            std::make_unique<UIManager>(*class_config.ui, class_config.runner.max_loop_rate);
        backends.push_back(ui_manager->diagnostics_backend());
    }

    if (class_config.publisher->uses_ros()) {
        auto ros_diag_publisher = std::make_shared<miniros::Publisher>(
            nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100));
        backends.push_back(
            std::make_shared<RosDiagnosticsBackend>(ros_diag_publisher, mcap_recorder));
    }

    DiagnosticsLogger::initialize(backends);

    auto publisher = make_publisher(nh, *class_config.publisher, mcap_recorder);
    auto camera = make_rgbd_camera(*class_config.camera);
    auto field_model = make_mask_model(*class_config.field_model);
    auto robot_mask_model = make_robot_blob_model(*class_config.robot_mask_model);
    auto field_filter = make_field_filter(*class_config.field_filter);
    auto keypoint_model = make_keypoint_model(*class_config.keypoint_model);
    auto robot_filter = make_robot_filter(*class_config.robot_filter);
    auto target_selector = make_target_selector(*class_config.target_selector);
    auto navigation = make_navigation(*class_config.navigation);
    auto transmitter = make_transmitter(*class_config.transmitter);

    Runner runner(class_config.runner, camera, class_config.health, field_model, robot_mask_model,
                  field_filter, keypoint_model, robot_filter, target_selector, navigation,
                  transmitter, publisher, handle_system_action,
                  ui_manager ? ui_manager->ui_state() : nullptr, mcap_recorder);

    runner.initialize();

    if (ui_manager) g_quittables[g_quittables_count++] = ui_manager.get();

    std::signal(SIGINT, signal_quit);
    std::signal(SIGTERM, signal_quit);

    if (ui_manager) ui_manager->start();

    int result = runner.run();
    spdlog::warn("Runner returned with code {}", result);

    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);
    g_quittables_count = 0;
    // ui_manager destructor: request_stop + join
    return result;
}
