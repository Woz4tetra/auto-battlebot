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

    std::vector<CrsfParseResult> parse(const uint8_t* data, size_t length);
    std::vector<CrsfParseResult> parse(const std::vector<uint8_t>& data);

    uint64_t crc_valid_frames() const { return crc_valid_frames_; }

   private:
    std::vector<uint8_t> buffer_;
    uint64_t crc_valid_frames_ = 0;

    CrsfParseResult parse_single_packet(const uint8_t* frame, size_t frame_len);
};

uint8_t crsf_crc8(const uint8_t* data, size_t length);

inline constexpr uint8_t kCrsfRadioAddress = 0xEA;

std::vector<uint8_t> encode_crsf_packet(const CrsfPacket& packet,
                                        uint8_t address = kCrsfRadioAddress);

}  // namespace auto_battlebot
