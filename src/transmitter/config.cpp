#include "transmitter/config.hpp"
#include "transmitter/noop_transmitter.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(TransmitterConfiguration, NoopTransmitterConfiguration, "NoopTransmitter")
    REGISTER_CONFIG(TransmitterConfiguration, PlaybackTransmitterConfiguration, "PlaybackTransmitter")
    REGISTER_CONFIG(TransmitterConfiguration, SimTransmitterConfiguration, "SimTransmitter")

    std::unique_ptr<TransmitterConfiguration> parse_transmitter_config(ConfigParser &parser)
    {
        return ConfigFactory<TransmitterConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for Transmitter" << std::endl;
        if (config.type == "NoopTransmitter")
        {
            return std::make_shared<NoopTransmitter>();
        }
        else if (config.type == "PlaybackTransmitter")
        {
            return std::make_shared<PlaybackTransmitter>(
                config_cast<PlaybackTransmitterConfiguration>(config));
        }
        else if (config.type == "PlaybackTransmitter")
        {
            return std::make_shared<SimTransmitter>(
                config_cast<SimTransmitterConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load Transmitter of type " + config.type);
    }
} // namespace auto_battlebot
