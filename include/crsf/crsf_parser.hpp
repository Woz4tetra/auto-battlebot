#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "crsf/crsf_packets.hpp"

namespace auto_battlebot {

struct CrsfParseError {
    std::string message;
};

using CrsfParseResult = std::pair<std::optional<CrsfPacket>, std::optional<CrsfParseError>>;

class CrsfParser {
   public:
    CrsfParser() = default;

    /** Append data to internal buffer, extract and return all complete valid packets. */
    std::vector<CrsfParseResult> parse(const uint8_t* data, size_t length);
    std::vector<CrsfParseResult> parse(const std::vector<uint8_t>& data);

   private:
    std::vector<uint8_t> buffer_;

    CrsfParseResult parse_single_packet(const uint8_t* frame, size_t frame_len);
};

uint8_t crsf_crc8(const uint8_t* data, size_t length);

}  // namespace auto_battlebot
