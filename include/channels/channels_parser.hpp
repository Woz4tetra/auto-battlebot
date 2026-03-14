#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace auto_battlebot {

static constexpr int kMaxChannels = 32;
static constexpr int kChannelsPerPacket = 16;

class ChannelsParser {
   public:
    ChannelsParser() = default;

    /**
     * Append data to internal buffer and extract complete channel arrays.
     * Returns one entry per complete 32-channel update received.
     */
    std::vector<std::array<int16_t, kMaxChannels>> process(const uint8_t* data, size_t length);
    std::vector<std::array<int16_t, kMaxChannels>> process(const std::vector<uint8_t>& data);

   private:
    std::vector<uint8_t> buffer_;
    std::optional<std::array<int16_t, kChannelsPerPacket>> phase0_;
    std::optional<std::array<int16_t, kChannelsPerPacket>> phase1_;

    std::optional<std::array<int16_t, kChannelsPerPacket>> parse_phase_packet(
        const uint8_t* packet_data, size_t packet_len, uint8_t phase);
};

}  // namespace auto_battlebot
