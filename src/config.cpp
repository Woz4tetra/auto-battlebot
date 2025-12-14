#include "config.hpp"
#include "config_parser.hpp"
#include "directories.hpp"
#include <fstream>
#include <iostream>
#include <toml++/toml.h>

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
            config.camera = parse_config_section<RgbdCameraConfiguration>(toml_data, "rgbd_camera", parsed_sections);
            config.field_model = parse_config_section<FieldModelConfiguration>(toml_data, "field_model", parsed_sections);
            config.field_filter = parse_config_section<FieldFilterConfiguration>(toml_data, "field_filter", parsed_sections);
            config.keypoint_model = parse_config_section<KeypointModelConfiguration>(toml_data, "keypoint_model", parsed_sections);
            config.robot_filter = parse_config_section<RobotFilterConfiguration>(toml_data, "robot_filter", parsed_sections);
            config.navigation = parse_config_section<NavigationConfiguration>(toml_data, "navigation", parsed_sections);
            config.transmitter = parse_config_section<TransmitterConfiguration>(toml_data, "transmitter", parsed_sections);

            // Validate no extra sections using the list we built during parsing
            validate_no_extra_sections(toml_data, parsed_sections, "classes.toml");
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
                        robot.label = string_to_label(label_str);
                        robot.group = string_to_group(group_str);
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

    std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const RgbdCameraConfiguration &config)
    {
        std::cerr << "Factory for camera type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config)
    {
        std::cerr << "Factory for field model type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config)
    {
        std::cerr << "Factory for field filter type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const KeypointModelConfiguration &config)
    {
        std::cerr << "Factory for keypoint model type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config)
    {
        std::cerr << "Factory for robot filter type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config)
    {
        std::cerr << "Factory for navigation type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config)
    {
        std::cerr << "Factory for transmitter type '" << config.type << "' not implemented" << std::endl;
        return nullptr;
    }

} // namespace auto_battlebot
