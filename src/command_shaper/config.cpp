#include "command_shaper/config.hpp"

#include <spdlog/spdlog.h>

#include "command_shaper/plant_command_shaper.hpp"
#include "config/config_cast.hpp"

namespace auto_battlebot {
REGISTER_CONFIG(CommandShaperConfiguration, PlantCommandShaperConfiguration, "PlantCommandShaper")

void PlantCommandShaperConfiguration::apply_plant(const PlantConfiguration &plant_config) {
    for (const auto &[name, gain] : {std::pair{"k_fwd", plant_config.params.k_fwd},
                                     std::pair{"k_rev", plant_config.params.k_rev},
                                     std::pair{"k_ang", plant_config.params.k_ang}}) {
        if (!(gain > 0.0)) {
            throw ConfigValidationError(std::string("PlantCommandShaper divides by [plant] ") +
                                        name + ", which must be > 0");
        }
    }
    plant = plant_config.params;
}

std::unique_ptr<CommandShaperConfiguration> load_command_shaper_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant) {
    std::unique_ptr<CommandShaperConfiguration> config;
    if (auto section = toml_data["command_shaper"].as_table()) {
        ConfigParser parser(*section, "command_shaper");
        config = ConfigFactory<CommandShaperConfiguration>::instance().create_and_parse(parser);
        parsed_sections.push_back("command_shaper");
    } else {
        config = std::make_unique<PlantCommandShaperConfiguration>();
    }
    config->apply_plant(plant);
    return config;
}

std::shared_ptr<CommandShaperInterface> make_command_shaper(
    const CommandShaperConfiguration &config) {
    spdlog::info("Selected {} for CommandShaper", config.type);
    if (config.type == "PlantCommandShaper") {
        const auto &plant_config = config_cast<PlantCommandShaperConfiguration>(config);
        return std::make_shared<PlantCommandShaper>(plant_config.plant,
                                                    plant_config.velocity_saturation_limit,
                                                    plant_config.linear_deadzone);
    }
    throw std::invalid_argument("Failed to load CommandShaper of type " + config.type);
}
}  // namespace auto_battlebot
