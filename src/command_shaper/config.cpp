#include "command_shaper/config.hpp"

#include <spdlog/spdlog.h>

#include "command_shaper/identity_command_shaper.hpp"
#include "config/config_cast.hpp"

namespace auto_battlebot {
REGISTER_CONFIG(CommandShaperConfiguration, IdentityCommandShaperConfiguration,
                "IdentityCommandShaper")

std::unique_ptr<CommandShaperConfiguration> load_command_shaper_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["command_shaper"].as_table();
    if (!section) {
        return std::make_unique<IdentityCommandShaperConfiguration>();
    }
    ConfigParser parser(*section, "command_shaper");
    auto config = ConfigFactory<CommandShaperConfiguration>::instance().create_and_parse(parser);
    parsed_sections.push_back("command_shaper");
    return config;
}

std::shared_ptr<CommandShaperInterface> make_command_shaper(
    const CommandShaperConfiguration &config) {
    spdlog::info("Selected {} for CommandShaper", config.type);
    if (config.type == "IdentityCommandShaper") {
        return std::make_shared<IdentityCommandShaper>(
            config_cast<IdentityCommandShaperConfiguration>(config).velocity_saturation_limit);
    }
    throw std::invalid_argument("Failed to load CommandShaper of type " + config.type);
}
}  // namespace auto_battlebot
