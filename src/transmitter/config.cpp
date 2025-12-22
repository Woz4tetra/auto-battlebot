#include "transmitter/config.hpp"
#include "transmitter/noop_transmitter.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(TransmitterConfiguration, NoopTransmitterConfiguration, "NoopTransmitter")
    REGISTER_CONFIG(TransmitterConfiguration, PlaybackTransmitterConfiguration, "PlaybackTransmitter")

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
            const auto &playback_config = static_cast<const PlaybackTransmitterConfiguration &>(config);
            return std::make_shared<PlaybackTransmitter>(playback_config.init_delay_seconds);
        }
        throw std::invalid_argument("Failed to load Transmitter of type " + config.type);
    }
} // namespace auto_battlebot
