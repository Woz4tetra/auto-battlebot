#include <miniros/ros.h>
#include <CLI/CLI.hpp>

#include "config/config.hpp"
#include "runner.hpp"
#include "diagnostics_logger/initialize_diagnostics_logger.hpp"

int main(int argc, char **argv)
{
    using namespace auto_battlebot;

    // Parse command line arguments
    CLI::App app{"Auto BattleBot - Autonomous robot control system"};
    std::string config_path = "";
    app.add_option("-c,--config", config_path, "Path to configuration directory")
        ->check(CLI::ExistingDirectory);

    try
    {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError &e)
    {
        return app.exit(e);
    }

    // Load configurations
    ClassConfiguration class_config = load_classes_from_config(config_path);
    std::vector<RobotConfig> robot_configs = load_robots_from_config(config_path);

    std::map<std::string, std::string> remappings;
    miniros::init(remappings, "auto_battlebot");

    miniros::NodeHandle nh;
    initialize_diagnostics_logger(nh);

    // Create interface instances using factory functions
    auto camera = make_rgbd_camera(*class_config.camera);
    auto field_model = make_field_model(*class_config.field_model);
    auto field_filter = make_field_filter(*class_config.field_filter);
    auto keypoint_model = make_keypoint_model(*class_config.keypoint_model);
    auto robot_filter = make_robot_filter(*class_config.robot_filter);
    auto navigation = make_navigation(*class_config.navigation);
    auto transmitter = make_transmitter(*class_config.transmitter);
    auto publisher = make_publisher(nh, *class_config.publisher);

    // Create runner with all components
    Runner runner(
        class_config.runner,
        robot_configs,
        camera,
        field_model,
        field_filter,
        keypoint_model,
        robot_filter,
        navigation,
        transmitter,
        publisher);

    // Initialize and run
    runner.initialize();
    return runner.run();
}
