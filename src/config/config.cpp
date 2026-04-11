#include "config/config.hpp"

#include <spdlog/spdlog.h>

namespace auto_battlebot {
template <typename ConfigType>
ConfigType parse_config_section(const toml::table &toml_data, const std::string &section_name,
                                std::vector<std::string> &parsed_sections) {
    ConfigType config;

    auto section = toml_data[section_name].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [" + section_name + "]");
    }

    ConfigParser parser(*section, section_name);
    config.type = parser.get_required_string("type");
    parser.validate_no_extra_fields();

    parsed_sections.push_back(section_name);
    return config;
}

ClassConfiguration load_classes_from_config(const std::string &config_path) {
    std::filesystem::path path =
        (config_path.empty() ? (get_config_dir() / "main") : std::filesystem::path(config_path)) /
        "classes.toml";
    std::string path_string = path.u8string();

    ClassConfiguration config;

    try {
        auto toml_data = toml::parse_file(path_string);

        std::vector<std::string> parsed_sections;

        config.camera = load_camera_from_toml(toml_data, parsed_sections);
        config.field_model = load_field_model_from_toml(toml_data, parsed_sections);
        config.robot_mask_model = load_robot_blob_model_from_toml(toml_data, parsed_sections);
        config.field_filter = load_field_filter_from_toml(toml_data, parsed_sections);
        config.keypoint_model = load_keypoint_model_from_toml(toml_data, parsed_sections);
        config.robot_filter = load_robot_filter_from_toml(toml_data, parsed_sections);
        config.target_selector = load_target_selector_from_toml(toml_data, parsed_sections);
        config.navigation = load_navigation_from_toml(toml_data, parsed_sections);
        config.transmitter = load_transmitter_from_toml(toml_data, parsed_sections);
        config.publisher = load_publisher_from_toml(toml_data, parsed_sections);
        config.ui = load_ui_from_toml(toml_data, parsed_sections);
        load_runner_from_toml(toml_data, parsed_sections, config.runner);
        config.mcap_recorder = load_mcap_config_from_toml(toml_data, parsed_sections);

        validate_no_extra_sections(toml_data, parsed_sections, path.stem());
        validate_no_extra_sections(toml_data, parsed_sections, path.stem());
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing classes config file: {}", path.string());
        spdlog::error("{}", e.description());
        throw;
    } catch (const ConfigValidationError &e) {
        spdlog::error("Configuration validation error in {}: {}", path.string(), e.what());
        throw;
    } catch (const std::exception &e) {
        spdlog::error("Error reading classes config file: {}", e.what());
        throw;
    }

    return config;
}

std::vector<RobotConfig> load_robots_from_config(const std::string &config_path) {
    std::filesystem::path path =
        (config_path.empty() ? (get_config_dir() / "main") : std::filesystem::path(config_path)) /
        "robots.toml";
    std::string path_string = path.u8string();

    std::vector<RobotConfig> robots;

    try {
        auto toml_data = toml::parse_file(path_string);

        if (auto robots_array = toml_data["robots"].as_array()) {
            for (const auto &robot_toml : *robots_array) {
                RobotConfig robot;
                if (auto robot_table = robot_toml.as_table()) {
                    std::string label_str = (*robot_table)["label"].value_or("");
                    std::string group_str = (*robot_table)["group"].value_or("");

                    auto label_opt = magic_enum::enum_cast<Label>(label_str);
                    if (label_opt.has_value()) {
                        robot.label = label_opt.value();
                    }

                    auto group_opt = magic_enum::enum_cast<Group>(group_str);
                    if (group_opt.has_value()) {
                        robot.group = group_opt.value();
                    }
                }
                robots.push_back(robot);
            }
        }
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing robot config file: {}", path.string());
        spdlog::error("{}", e.description());
        return robots;
    } catch (const std::exception &e) {
        spdlog::error("Error reading robot config file: {}", e.what());
        return robots;
    }

    return robots;
}
}  // namespace auto_battlebot
