#include "transmitter/config.hpp"

#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "transmitter/noop_transmitter.hpp"
#include "transmitter/opentx_transmitter.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(TransmitterConfiguration, NoopTransmitterConfiguration, "NoopTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, PlaybackTransmitterConfiguration, "PlaybackTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, SimTransmitterConfiguration, "SimTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, OpenTxTransmitterConfiguration, "OpenTxTransmitter")

std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser) {
    return ConfigFactory<TransmitterConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<TransmitterConfiguration> load_transmitter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["transmitter"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [transmitter]");
    }
    ConfigParser parser(*section, "transmitter");
    auto config = parse_transmitter_config(parser);
    parsed_sections.push_back("transmitter");
    return config;
}

std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config) {
    std::cout << "Selected " + config.type + " for Transmitter" << std::endl;
    if (config.type == "NoopTransmitter") {
        return std::make_shared<NoopTransmitter>();
    } else if (config.type == "PlaybackTransmitter") {
        return std::make_shared<PlaybackTransmitter>(
            config_cast<PlaybackTransmitterConfiguration>(config));
    } else if (config.type == "SimTransmitter") {
        return std::make_shared<SimTransmitter>(config_cast<SimTransmitterConfiguration>(config));
    } else if (config.type == "OpenTxTransmitter") {
        return std::make_shared<OpenTxTransmitter>(
            config_cast<OpenTxTransmitterConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load Transmitter of type " + config.type);
}
}  // namespace auto_battlebot
