#include "channels/channels_parser.hpp"

#include <cstring>

namespace auto_battlebot {

static constexpr uint8_t kSyncSequence[] = {0xA3, 0xA4, 0xA5};
static constexpr size_t kSyncLen = sizeof(kSyncSequence);
// Each phase packet: phase(1) + length(1) + 16*int16_le(32) + checksum(1) = 35 bytes
static constexpr uint8_t kExpectedDataLength = kChannelsPerPacket * 2 + 1;  // 33

std::optional<std::array<int16_t, kChannelsPerPacket>> ChannelsParser::parse_phase_packet(
    const uint8_t* packet_data, size_t packet_len, uint8_t phase) {
    // packet_data: [phase][length][channel_data x 32 bytes][checksum]
    if (packet_len < 2) return std::nullopt;

    uint8_t length = packet_data[1];
    if (length != kExpectedDataLength) return std::nullopt;
    if (packet_len < static_cast<size_t>(2 + length)) return std::nullopt;

    // Verify XOR checksum: phase ^ length ^ XOR(all data bytes)
    uint8_t expected_checksum = phase ^ length;
    for (size_t i = 2; i < static_cast<size_t>(2 + length - 1); ++i) {
        expected_checksum ^= packet_data[i];
    }
    uint8_t received_checksum = packet_data[2 + length - 1];
    if (expected_checksum != received_checksum) return std::nullopt;

    // Unpack 16 x little-endian int16
    std::array<int16_t, kChannelsPerPacket> channels{};
    const uint8_t* channel_data = packet_data + 2;
    for (int i = 0; i < kChannelsPerPacket; ++i) {
        int16_t value;
        std::memcpy(&value, channel_data + i * 2, sizeof(int16_t));
        channels[i] = value;
    }
    return channels;
}

std::vector<std::array<int16_t, kMaxChannels>> ChannelsParser::process(const uint8_t* data,
                                                                       size_t length) {
    buffer_.insert(buffer_.end(), data, data + length);

    std::vector<std::array<int16_t, kMaxChannels>> results;

    while (buffer_.size() >= kSyncLen + 2) {
        // Find sync sequence
        size_t sync_pos = buffer_.size();
        for (size_t i = 0; i + kSyncLen <= buffer_.size(); ++i) {
            if (buffer_[i] == kSyncSequence[0] && buffer_[i + 1] == kSyncSequence[1] &&
                buffer_[i + 2] == kSyncSequence[2]) {
                sync_pos = i;
                break;
            }
        }

        if (sync_pos == buffer_.size()) {
            buffer_.clear();
            break;
        }

        if (sync_pos > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<ptrdiff_t>(sync_pos));
        }

        if (buffer_.size() < kSyncLen + 2) break;

        uint8_t phase = buffer_[kSyncLen];
        uint8_t packet_length = buffer_[kSyncLen + 1];
        size_t total_size = kSyncLen + 2 + packet_length;

        if (buffer_.size() < total_size) break;

        // packet_data starts at sync_len (phase byte), length = 2 + packet_length
        auto channels = parse_phase_packet(buffer_.data() + kSyncLen, 2 + packet_length, phase);

        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<ptrdiff_t>(total_size));

        if (!channels) continue;

        if (phase == 0) {
            phase0_ = channels;
        } else if (phase == 1) {
            phase1_ = channels;
        } else {
            continue;
        }

        if (phase0_ && phase1_) {
            std::array<int16_t, kMaxChannels> complete{};
            for (int i = 0; i < kChannelsPerPacket; ++i) {
                complete[i] = (*phase0_)[i];
                complete[kChannelsPerPacket + i] = (*phase1_)[i];
            }
            phase0_.reset();
            phase1_.reset();
            results.push_back(complete);
        }
    }

    return results;
}

std::vector<std::array<int16_t, kMaxChannels>> ChannelsParser::process(
    const std::vector<uint8_t>& data) {
    return process(data.data(), data.size());
}

}  // namespace auto_battlebot
