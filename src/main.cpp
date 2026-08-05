#include <miniros/ros.h>
#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <CLI/CLI.hpp>
#include <csignal>
#include <diagnostic_msgs/DiagnosticArray.hxx>
#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

#include "config/config.hpp"
#include "config/profile_selection.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/ros_diagnostics_backend.hpp"
#include "directories.hpp"
#include "health/health_logger.hpp"
#include "logging/logging.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "perception_batch/parallel_model_batch.hpp"
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
    bool print_config = false;
    app.add_flag("--print-config", print_config,
                 "Print the resolved config values (extends chain merged) and exit");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    ProfileSelectorConfig profile_selector = load_profile_selector(get_config_dir());
    std::vector<std::string> available_profiles =
        list_available_profiles(get_config_dir(), profile_selector.pattern);

    std::filesystem::path config_path;
    std::string active_profile;
    if (config_path_string.empty()) {
        // Service launch: use the profile the operator selected in the UI, falling back to the
        // repo default. An explicit -c (dev/playback/sim) bypasses the selector entirely.
        try {
            active_profile = resolve_active_profile(get_config_dir(), profile_selector);
        } catch (const std::exception& e) {
            spdlog::error("Could not resolve a config profile: {}", e.what());
            return 1;
        }
        config_path = get_config_dir() / (active_profile + ".toml");
    } else {
        config_path = normalize_config_path(config_path_string);
        std::error_code ec;
        std::filesystem::path rel = std::filesystem::relative(config_path, get_config_dir(), ec);
        active_profile = (!ec && !rel.empty()) ? rel.replace_extension().generic_string()
                                               : config_path.stem().string();
    }
    if (print_config) {
        try {
            toml::table merged = load_merged_config(config_path);
            std::cout << "# Resolved config for profile '" << active_profile << "'\n";
            std::cout << "# Source: " << config_path.string() << "\n";
            std::cout << "# Note: shows values set by the config and its extends bases; defaults\n";
            std::cout << "#       applied in code for omitted optional fields are not shown.\n\n";
            std::cout << merged << "\n";
        } catch (const std::exception& e) {
            spdlog::error("Failed to load config '{}': {}", config_path.string(), e.what());
            return 1;
        }
        return 0;
    }

    ClassConfiguration class_config = load_classes_from_config(config_path);

    auto mcap_recorder = make_mcap_recorder(class_config.mcap_recorder, active_profile);
    setup_logging(mcap_recorder);
    std::map<std::string, std::string> remappings;
    miniros::init(remappings, "auto_battlebot");
    miniros::NodeHandle nh;
    setup_rosout_publisher(nh);

    std::unique_ptr<UIManager> ui_manager;
    std::vector<std::shared_ptr<DiagnosticsBackend>> backends;

    if (class_config.ui && class_config.ui->enable) {
        ui_manager =
            std::make_unique<UIManager>(*class_config.ui, class_config.runner.max_loop_rate,
                                        available_profiles, active_profile);
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
    auto perception_batch = std::make_shared<ParallelModelBatch>(keypoint_model, robot_mask_model);
    auto clock = make_clock(*class_config.clock);
    auto robot_filter = make_robot_filter(*class_config.robot_filter, clock);
    auto target_selector = make_target_selector(*class_config.target_selector);
    auto navigation = make_navigation(*class_config.navigation, clock);
    auto transmitter = make_transmitter(*class_config.transmitter, clock);
    auto health_logger = std::make_shared<HealthLogger>(class_config.health);

    Runner runner(
        class_config.runner, camera, health_logger, field_model, robot_mask_model, field_filter,
        keypoint_model, perception_batch, robot_filter, target_selector, navigation, transmitter,
        publisher, handle_system_action,
        [profile_selector](const std::string& name) {
            write_selection_file(profile_selector, name);
        },
        ui_manager ? ui_manager->ui_state() : nullptr, mcap_recorder, clock);

    runner.initialize();

    // The runner is registered unconditionally. Registering only the UI manager left
    // ui.enable = false with no quittables at all, so SIGINT and SIGTERM did nothing and
    // the process could only be stopped with SIGKILL.
    g_quittables[g_quittables_count++] = &runner;
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
