#pragma once

#include <toml++/toml.h>

#include <memory>
#include <string>
#include <vector>

#include "command_shaper/command_shaper_interface.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "plant/config.hpp"

namespace auto_battlebot {
struct CommandShaperConfiguration {
    std::string type;
    virtual ~CommandShaperConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    virtual void apply_plant([[maybe_unused]] const PlantConfiguration &plant) {}
};

struct PlantCommandShaperConfiguration : public CommandShaperConfiguration {
    double velocity_saturation_limit = 1.0;
    JigPlantParams plant;

    PlantCommandShaperConfiguration() { type = "PlantCommandShaper"; }

    PARSE_CONFIG_FIELDS(PARSE_FIELD_DOUBLE(velocity_saturation_limit))

    void apply_plant(const PlantConfiguration &plant_config) override;
};

std::shared_ptr<CommandShaperInterface> make_command_shaper(
    const CommandShaperConfiguration &config);
std::unique_ptr<CommandShaperConfiguration> load_command_shaper_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant);
}  // namespace auto_battlebot
