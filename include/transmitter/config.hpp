#pragma once

#include "data_structures.hpp"

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
} // namespace auto_battlebot
