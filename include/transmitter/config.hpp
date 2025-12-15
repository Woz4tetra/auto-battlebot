#pragma once

#include "data_structures.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot
{
    struct TransmitterConfiguration
    {
        std::string type;
    };

    struct NoopTransmitterConfiguration : public TransmitterConfiguration
    {
        NoopTransmitterConfiguration()
        {
            type = "NoopTransmitter";
        }
    };

    std::shared_ptr<TransmitterInterface> make_transmitter(const TransmitterConfiguration &config);
} // namespace auto_battlebot
