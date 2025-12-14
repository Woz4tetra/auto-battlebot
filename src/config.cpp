#include "config.hpp"
#include <fstream>
#include <iostream>
#include <toml++/toml.h>

namespace auto_battlebot
{
    ClassConfiguration load_classes_from_config(const std::string &config_path)
    {
        std::string path = config_path.empty() ? "config/classes.toml" : config_path;

        ClassConfiguration config;

        auto toml_data = toml::parse_file(path);

        config.camera = toml_data["camera"].value_or("realsense");
        config.field_model = toml_data["field_model"].value_or("default");
        config.field_filter = toml_data["field_filter"].value_or("default");
        config.keypoint_model = toml_data["keypoint_model"].value_or("default");
        config.robot_filter = toml_data["robot_filter"].value_or("default");
        config.navigation = toml_data["navigation"].value_or("default");
        config.transmitter = toml_data["transmitter"].value_or("default");

        return config;
    }

    std::vector<RobotConfig> load_robots_from_config(const std::string &config_path)
    {
        std::string path = config_path.empty() ? "config/robots.toml" : config_path;

        std::vector<RobotConfig> robots;

        try
        {
            auto toml_data = toml::parse_file(path);

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
            std::cerr << "Error parsing robot config file: " << path << std::endl;
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

    std::shared_ptr<RgbdCameraInterface> make_rgbd_camera(const std::string &camera_type)
    {
        std::cerr << "Factory for camera type '" << camera_type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<FieldModelInterface> make_field_model(const std::string &model_type)
    {
        std::cerr << "Factory for field model type '" << model_type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<FieldFilterInterface> make_field_filter(const std::string &filter_type)
    {
        std::cerr << "Factory for field filter type '" << filter_type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<KeypointModelInterface> make_keypoint_model(const std::string &model_type)
    {
        std::cerr << "Factory for keypoint model type '" << model_type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const std::string &filter_type)
    {
        std::cerr << "Factory for robot filter type '" << filter_type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<NavigationInterface> make_navigation(const std::string &navigation_type)
    {
        std::cerr << "Factory for navigation type '" << navigation_type << "' not implemented" << std::endl;
        return nullptr;
    }

    std::shared_ptr<TransmitterInterface> make_transmitter(const std::string &transmitter_type)
    {
        std::cerr << "Factory for transmitter type '" << transmitter_type << "' not implemented" << std::endl;
        return nullptr;
    }

} // namespace auto_battlebot
