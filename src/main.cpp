#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <CLI/CLI.hpp>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "battery/battery_monitor.hpp"
#include "config/config.hpp"
#include "config/profile_selection.hpp"
#include "control_loop/config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/foxglove_diagnostics_backend.hpp"
#include "directories.hpp"
#include "health/health_logger.hpp"
#include "host/host_services.hpp"
#include "host/system_actions.hpp"
#include "keypoint_filter/height_gate.hpp"
#include "keypoint_filter/static_gate.hpp"
#include "logging/logging.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "perception_batch/parallel_model_batch.hpp"
#include "publisher/config.hpp"
#include "quittable.hpp"
#include "remote/command_decode.hpp"
#include "remote/status_bus.hpp"
#include "runner.hpp"

namespace {
constexpr std::size_t kMaxQuittables = 8;
auto_battlebot::Quittable* g_quittables[kMaxQuittables] = {};
std::size_t g_quittables_count = 0;

void signal_quit(int) {
    for (std::size_t i = 0; i < g_quittables_count; ++i) g_quittables[i]->request_quit();
}

// The build version stamped next to the executable by the generate_git_version target (see
// cmake/git_version.cmake). Kept out of the binary so a new commit only rewrites the text file
// instead of forcing a recompile and relink.
std::string read_build_version() {
    std::error_code ec;
    const std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) return "unknown";
    std::ifstream file(exe.parent_path() / "git_version.txt");
    std::string version;
    if (!file || !std::getline(file, version) || version.empty()) return "unknown";
    return version;
}

// Owns every component's lifetime. A reboot or power-off command stops the runner loop, and the
// host command runs here, once the loop is down but before the remaining components are
// destroyed.
int run_application(const auto_battlebot::ClassConfiguration& class_config,
                    const std::string& active_profile,
                    const std::vector<std::string>& available_profiles,
                    const auto_battlebot::ProfileSelectorConfig& profile_selector) {
    using namespace auto_battlebot;

    // Set by the Runner when a command asks to reboot or power off the host.
    std::optional<SystemAction> pending_system_action;

    // Before the recorder picks a path: a replay must not be able to record over the file it
    // is reading.
    reserve_camera_input_paths(*class_config.camera);
    auto mcap_recorder = make_mcap_recorder(class_config.mcap_recorder, active_profile);
    setup_logging(mcap_recorder);

    std::vector<std::shared_ptr<DiagnosticsBackend>> backends;

    // Never blocks: a missing relay just means no live viz until it comes back.
    auto viz_sink = make_viz_sink(*class_config.publisher);
    if (viz_sink) attach_log_viz_sink(viz_sink);
    if (viz_sink || mcap_recorder) {
        backends.push_back(std::make_shared<FoxgloveDiagnosticsBackend>(viz_sink, mcap_recorder));
    }

    DiagnosticsLogger::initialize(backends);

    auto publisher = make_publisher(*class_config.publisher, viz_sink, mcap_recorder);
    auto camera = make_rgbd_camera(*class_config.camera, mcap_recorder);
    auto field_model = make_mask_model(*class_config.field_model);
    auto robot_mask_model = make_robot_blob_model(*class_config.robot_mask_model);
    auto field_filter = make_field_filter(*class_config.field_filter);
    auto keypoint_model =
        make_keypoint_model(*class_config.keypoint_model, viz_sink, mcap_recorder);
    // TODO make a NoopModelBatch that is set when parallel_models is false.
    auto perception_batch = std::make_shared<ParallelModelBatch>(keypoint_model, robot_mask_model);
    auto clock = make_clock(*class_config.clock);
    auto robot_filter = make_robot_filter(*class_config.robot_filter, clock);
    auto target_selector = make_target_selector(*class_config.target_selector);
    auto navigation = make_navigation(*class_config.navigation, clock);
    auto transmitter = make_transmitter(*class_config.transmitter, clock, viz_sink, mcap_recorder);
    auto health_logger = std::make_shared<HealthLogger>(class_config.health);
    auto height_gate = std::make_shared<KeypointHeightGate>(class_config.keypoint_filter.height);
    auto static_gate =
        std::make_shared<StaticDetectionGate>(class_config.keypoint_filter.static_gate);

    // The control loop owns the filter/target/navigation/transmit half. A threaded driver runs it
    // on its own thread, so nothing else may touch those components after Runner::initialize().
    auto hazard_assembler = make_hazard_assembler(*class_config.field_filter, clock);
    auto control_loop_body = std::make_shared<ControlLoop>(
        robot_filter, target_selector, navigation, transmitter, clock, hazard_assembler);
    auto control_loop = make_control_loop(*class_config.control_loop, control_loop_body);

    // Commands come from the viz sink: the web page and Foxglove.
    RunnerRemote remote;
    if (viz_sink || mcap_recorder) {
        remote.status = std::make_shared<remote::StatusBus>(viz_sink, mcap_recorder);
    }
    remote.host = std::make_shared<HostServices>(HostServices::Options{}, remote.status);
    // Created after DiagnosticsLogger::initialize, so its battery_ina219 diagnostics have a
    // backend.
    auto battery_monitor = make_battery_monitor(class_config.battery, remote.status);
    remote.app_info.available_profiles = available_profiles;
    remote.app_info.current_profile = active_profile;
    remote.app_info.max_loop_rate_hz = class_config.runner.max_loop_rate;
    remote.app_info.version = read_build_version();
    if (class_config.ui) {
        remote.app_info.rate_fail_threshold = class_config.ui->rate_fail_threshold;
        remote.app_info.rate_fail_duration_sec = class_config.ui->rate_fail_duration_sec;
        for (const auto& [label, color] : class_config.ui->label_colors) {
            remote.app_info.label_colors.push_back({.label = label, .color = color});
        }
    }
    auto command_queue = remote.commands;

    Runner runner(
        class_config.runner, camera, health_logger, field_model, robot_mask_model, field_filter,
        keypoint_model, height_gate, static_gate, perception_batch, control_loop, publisher,
        [&pending_system_action](SystemAction action) { pending_system_action = action; },
        [profile_selector](const std::string& name) {
            write_selection_file(profile_selector, name);
        },
        mcap_recorder, clock, std::move(remote));

    runner.initialize();

    // Runner::request_quit() only stores atomics, so it is safe from the signal handler.
    g_quittables[g_quittables_count++] = &runner;

    std::signal(SIGINT, signal_quit);
    std::signal(SIGTERM, signal_quit);

    if (viz_sink) {
        viz_sink->set_client_message_handler(
            [command_queue](const std::string& topic, const std::byte* data, size_t len) {
                if (auto command = remote::decode_command(topic, data, len)) {
                    command_queue->post(std::move(*command));
                }
            });
    }

    int result = runner.run();
    spdlog::warn("Runner returned with code {}", result);
    // The sink outlives this scope (the log sink holds it); drop the handler before runner goes.
    if (viz_sink) viz_sink->set_client_message_handler(nullptr);

    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);
    g_quittables_count = 0;

    // Run the host command here rather than after this function returns. The camera and
    // model destructors run on the way out, and a ZED close or a TensorRT teardown that
    // blocks or faults would swallow the reboot entirely. The loop is stopped and both
    // recordings are closed by now, so there is nothing left to lose if systemd tears the
    // process down in the middle of those destructors.
    if (pending_system_action) {
        // Logging is asynchronous; get the shutdown lines out before the host goes down.
        spdlog::default_logger()->flush();
        handle_system_action(*pending_system_action);
    }
    return result;
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
        // Service launch: use the profile the operator selected on the web page, falling back to
        // the repo default. An explicit -c (dev/playback/sim) bypasses the selector entirely.
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

    try {
        ClassConfiguration class_config = load_classes_from_config(config_path);
        const int result =
            run_application(class_config, active_profile, available_profiles, profile_selector);
        shutdown_logging();
        return result;
    } catch (const std::exception& e) {
        // Startup failures that leave the stack unusable (a bad config, a model whose engine
        // does not load) throw. Catching here exits with a message and a nonzero code instead
        // of terminating on the uncaught exception.
        spdlog::error("Fatal: {}", e.what());
        shutdown_logging();
        return 1;
    }
}
