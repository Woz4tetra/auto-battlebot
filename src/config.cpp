#include "config.hpp"
#include "config_parser.hpp"
#include "directories.hpp"
#include <fstream>
#include <iostream>
#include <toml++/toml.h>
#include <magic_enum.hpp>

namespace auto_battlebot
{
    template <typename ConfigType>
    ConfigType parse_config_section(const toml::table &toml_data, const std::string &section_name, std::vector<std::string> &parsed_sections)
    {
        ConfigType config;

        auto section = toml_data[section_name].as_table();
        if (!section)
        {
            throw ConfigValidationError("Missing required section [" + section_name + "]");
        }

        ConfigParser parser(*section, section_name);
        config.type = parser.get_required_string("type");
        parser.validate_no_extra_fields();

        parsed_sections.push_back(section_name);
        return config;
    }

    ClassConfiguration load_classes_from_config(const std::string &config_path)
    {
        std::filesystem::path path = config_path.empty()
                                         ? get_config_dir() / "classes.toml"
                                         : std::filesystem::path(config_path);
        std::string path_string = path.u8string();

        ClassConfiguration config;

        try
        {
            auto toml_data = toml::parse_file(path_string);

            std::vector<std::string> parsed_sections;

            // Parse all sections with polymorphic configs
            auto camera_section = toml_data["rgbd_camera"].as_table();
            if (!camera_section)
            {
                throw ConfigValidationError("Missing required section [rgbd_camera]");
            }
            ConfigParser camera_parser(*camera_section, "rgbd_camera");
            config.camera = parse_rgbd_camera_config(camera_parser);
            parsed_sections.push_back("rgbd_camera");

            auto field_model_section = toml_data["field_model"].as_table();
            if (!field_model_section)
            {
                throw ConfigValidationError("Missing required section [field_model]");
            }
            ConfigParser field_model_parser(*field_model_section, "field_model");
            config.field_model = parse_field_model_config(field_model_parser);
            parsed_sections.push_back("field_model");

            auto field_filter_section = toml_data["field_filter"].as_table();
            if (!field_filter_section)
            {
                throw ConfigValidationError("Missing required section [field_filter]");
            }
            ConfigParser field_filter_parser(*field_filter_section, "field_filter");
            config.field_filter = parse_field_filter_config(field_filter_parser);
            parsed_sections.push_back("field_filter");

            auto keypoint_model_section = toml_data["keypoint_model"].as_table();
            if (!keypoint_model_section)
            {
                throw ConfigValidationError("Missing required section [keypoint_model]");
            }
            ConfigParser keypoint_model_parser(*keypoint_model_section, "keypoint_model");
            config.keypoint_model = parse_keypoint_model_config(keypoint_model_parser);
            parsed_sections.push_back("keypoint_model");

            auto robot_filter_section = toml_data["robot_filter"].as_table();
            if (!robot_filter_section)
            {
                throw ConfigValidationError("Missing required section [robot_filter]");
            }
            ConfigParser robot_filter_parser(*robot_filter_section, "robot_filter");
            config.robot_filter = parse_robot_filter_config(robot_filter_parser);
            parsed_sections.push_back("robot_filter");

            auto navigation_section = toml_data["navigation"].as_table();
            if (!navigation_section)
            {
                throw ConfigValidationError("Missing required section [navigation]");
            }
            ConfigParser navigation_parser(*navigation_section, "navigation");
            config.navigation = parse_navigation_config(navigation_parser);
            parsed_sections.push_back("navigation");

            auto transmitter_section = toml_data["transmitter"].as_table();
            if (!transmitter_section)
            {
                throw ConfigValidationError("Missing required section [transmitter]");
            }
            ConfigParser transmitter_parser(*transmitter_section, "transmitter");
            config.transmitter = parse_transmitter_config(transmitter_parser);
            parsed_sections.push_back("transmitter");

            // Validate no extra sections using the list we built during parsing
            validate_no_extra_sections(toml_data, parsed_sections, path.stem());
        }
        catch (const toml::parse_error &e)
        {
            std::cerr << "Error parsing classes config file: " << path.string() << std::endl;
            std::cerr << e.description() << std::endl;
            throw;
        }
        catch (const ConfigValidationError &e)
        {
            std::cerr << "Configuration validation error in " << path.string() << ":" << std::endl;
            std::cerr << e.what() << std::endl;
            throw;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading classes config file: " << e.what() << std::endl;
            throw;
        }

        return config;
    }

    std::vector<RobotConfig> load_robots_from_config(const std::string &config_path)
    {
        std::filesystem::path path = config_path.empty()
                                         ? get_config_dir() / "robots.toml"
                                         : std::filesystem::path(config_path);
        std::string path_string = path.u8string();

        std::vector<RobotConfig> robots;

        try
        {
            auto toml_data = toml::parse_file(path_string);

            if (auto robots_array = toml_data["robots"].as_array())
            {
                for (const auto &robot_toml : *robots_array)
                {
                    RobotConfig robot;
                    if (auto robot_table = robot_toml.as_table())
                    {
                        std::string label_str = (*robot_table)["label"].value_or("");
                        std::string group_str = (*robot_table)["group"].value_or("");

                        auto label_opt = magic_enum::enum_cast<Label>(label_str);
                        if (label_opt.has_value())
                        {
                            robot.label = label_opt.value();
                        }

                        auto group_opt = magic_enum::enum_cast<Group>(group_str);
                        if (group_opt.has_value())
                        {
                            robot.group = group_opt.value();
                        }
                    }
                    robots.push_back(robot);
                }
            }
        }
        catch (const toml::parse_error &e)
        {
            std::cerr << "Error parsing robot config file: " << path.string() << std::endl;
            std::cerr << e.description() << std::endl;
            return robots;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading robot config file: " << e.what() << std::endl;
            return robots;
        }

        return robots;
    }
} // namespace auto_battlebot
