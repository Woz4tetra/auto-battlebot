#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "config/camera_profile_store.hpp"
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
#include "publisher/config.hpp"
#include "quittable.hpp"
#include "remote/command_decode.hpp"
#include "remote/status_bus.hpp"
#include "runner.hpp"
#include "signal_quit.hpp"

namespace {
std::string read_build_version() {
    std::error_code ec;
    const std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) return "unknown";
    std::ifstream file(exe.parent_path() / "git_version.txt");
    std::string version;
    if (!file || !std::getline(file, version) || version.empty()) return "unknown";
    return version;
}

std::shared_ptr<auto_battlebot::McapRecorder> make_recorder_clear_of_replay_inputs(
    const auto_battlebot::ClassConfiguration& class_config, const std::string& active_profile) {
    auto_battlebot::reserve_camera_input_paths(*class_config.camera);
    return auto_battlebot::make_mcap_recorder(class_config.mcap_recorder, active_profile);
}

void run_host_command_before_teardown(
    const std::optional<auto_battlebot::SystemAction>& pending_system_action) {
    if (!pending_system_action) return;
    spdlog::default_logger()->flush();
    auto_battlebot::handle_system_action(*pending_system_action);
}

int run_application(const auto_battlebot::ClassConfiguration& class_config,
                    const auto_battlebot::ResolvedConfig& resolved_config,
                    const std::string& active_profile,
                    const std::vector<std::string>& available_profiles,
                    const auto_battlebot::ProfileSelectorConfig& profile_selector) {
    using namespace auto_battlebot;

    std::optional<SystemAction> pending_system_action;

    setup_logging();
    auto mcap_recorder = make_recorder_clear_of_replay_inputs(class_config, active_profile);
    if (mcap_recorder) {
        mcap_recorder->write_metadata(kResolvedConfigMetadataKey,
                                      format_resolved_toml(resolved_config));
    }
    attach_log_recorder(mcap_recorder);

    std::vector<std::shared_ptr<DiagnosticsBackend>> backends;

    auto viz_sink = make_viz_sink(*class_config.publisher);
    if (viz_sink) attach_log_viz_sink(viz_sink);
    if (viz_sink || mcap_recorder) {
        backends.push_back(std::make_shared<FoxgloveDiagnosticsBackend>(viz_sink, mcap_recorder));
    }

    DiagnosticsLogger::initialize(backends);

    auto publisher = make_publisher(*class_config.publisher, viz_sink, mcap_recorder);
    auto camera = make_rgbd_camera(*class_config.camera, mcap_recorder);
    auto field_model = make_mask_model(*class_config.field_model);
    auto field_filter = make_field_filter(*class_config.field_filter);
    auto keypoint_model =
        make_keypoint_model(*class_config.keypoint_model, viz_sink, mcap_recorder);
    auto clock = make_clock(*class_config.clock);
    auto robot_filter = make_robot_filter(*class_config.robot_filter, clock);
    auto target_selector = make_target_selector(*class_config.target_selector);
    auto navigation = make_navigation(*class_config.navigation, clock);
    auto transmitter = make_transmitter(*class_config.transmitter, clock, viz_sink, mcap_recorder);
    auto health_logger = std::make_shared<HealthLogger>(class_config.health);
    auto height_gate = std::make_shared<KeypointHeightGate>(class_config.keypoint_filter.height);
    auto static_gate =
        std::make_shared<StaticDetectionGate>(class_config.keypoint_filter.static_gate);

    auto hazard_assembler = make_hazard_assembler(*class_config.field_filter, clock);
    auto control_loop_body = std::make_shared<ControlLoop>(
        robot_filter, target_selector, navigation, transmitter, clock, hazard_assembler,
        class_config.control_loop->robot_cache_max_age_s);
    auto control_loop = make_control_loop(*class_config.control_loop, control_loop_body);

    RunnerRemote remote;
    if (viz_sink || mcap_recorder) {
        remote.status = std::make_shared<remote::StatusBus>(viz_sink, mcap_recorder);
    }
    remote.host = std::make_shared<HostServices>(HostServices::Options{}, remote.status);
    remote.camera_profiles =
        std::make_shared<CameraProfileStore>(expand_home(CameraProfileStore::kDefaultPath));
    remote.camera_profiles->load();
    remote.app_info.available_profiles = available_profiles;
    remote.app_info.current_profile = active_profile;
    remote.app_info.max_loop_rate_hz = class_config.runner.max_loop_rate_hz;
    remote.app_info.version = read_build_version();
    remote.resolved_config.profile = active_profile;
    for (const auto& field : resolved_config.fields) {
        remote.resolved_config.fields.push_back({.section = field.section,
                                                 .key = field.key,
                                                 .value = field.value,
                                                 .source = field.source});
    }
    if (class_config.ui) {
        remote.app_info.rate_fail_threshold = class_config.ui->rate_fail_threshold;
        remote.app_info.rate_fail_duration_sec = class_config.ui->rate_fail_duration_sec;
        for (const auto& [label, color] : class_config.ui->label_colors) {
            remote.app_info.label_colors.push_back({.label = label, .color = color});
        }
    }
    auto command_queue = remote.commands;

    Runner runner(
        class_config.runner, camera, health_logger, field_model, field_filter, keypoint_model,
        height_gate, static_gate, control_loop, publisher,
        [&pending_system_action](SystemAction action) { pending_system_action = action; },
        [profile_selector](const std::string& name) {
            write_selection_file(profile_selector, name);
        },
        mcap_recorder, clock, std::move(remote));

    runner.initialize();

    std::optional<ScopedSignalQuit> signal_quit;
    signal_quit.emplace(runner);

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
    if (viz_sink) viz_sink->set_client_message_handler(nullptr);

    signal_quit.reset();

    run_host_command_before_teardown(pending_system_action);
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
    app.add_flag(
        "--print-config", print_config,
        "Print every config field with its value and source file:line or default, then exit");

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
            ResolvedConfig resolved;
            load_classes_from_config(config_path, {}, &resolved);
            std::cout << "# Resolved config for profile '" << active_profile << "'\n\n";
            std::cout << format_resolved_toml(resolved);
        } catch (const std::exception& e) {
            spdlog::error("Failed to load config '{}': {}", config_path.string(), e.what());
            return 1;
        }
        return 0;
    }

    try {
        ResolvedConfig resolved_config;
        ClassConfiguration class_config =
            load_classes_from_config(config_path, {}, &resolved_config);
        const int result = run_application(class_config, resolved_config, active_profile,
                                           available_profiles, profile_selector);
        shutdown_logging();
        return result;
    } catch (const std::exception& e) {
        spdlog::error("Fatal: {}", e.what());
        shutdown_logging();
        return 1;
    }
}
