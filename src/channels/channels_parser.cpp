#include "channels/channels_parser.hpp"

#include <cstring>

namespace auto_battlebot {

static constexpr uint8_t kSyncSequence[] = {0xA3, 0xA4, 0xA5};
static constexpr size_t kSyncLen = sizeof(kSyncSequence);

static constexpr size_t kChannelDataBytes = kChannelsPerPacket * 2;
static constexpr uint8_t kExpectedDataLength = kChannelDataBytes + 1;
static constexpr size_t kPhasePacketBytes = 2 + kExpectedDataLength;

static bool xor_checksum_matches(const uint8_t* packet_data, uint8_t phase, uint8_t length) {
    uint8_t expected_checksum = phase ^ length;
    for (size_t i = 2; i < static_cast<size_t>(2 + length - 1); ++i) {
        expected_checksum ^= packet_data[i];
    }
    return expected_checksum == packet_data[2 + length - 1];
}

static std::array<int16_t, kChannelsPerPacket> unpack_le_int16_channels(
    const uint8_t* channel_data) {
    std::array<int16_t, kChannelsPerPacket> channels{};
    for (int i = 0; i < kChannelsPerPacket; ++i) {
        int16_t value;
        std::memcpy(&value, channel_data + i * 2, sizeof(int16_t));
        channels[i] = value;
    }
    return channels;
}

static size_t find_sync_sequence(const std::vector<uint8_t>& buffer) {
    for (size_t i = 0; i + kSyncLen <= buffer.size(); ++i) {
        if (buffer[i] == kSyncSequence[0] && buffer[i + 1] == kSyncSequence[1] &&
            buffer[i + 2] == kSyncSequence[2]) {
            return i;
        }
    }
    return buffer.size();
}

std::optional<std::array<int16_t, kChannelsPerPacket>> ChannelsParser::parse_phase_packet(
    const uint8_t* packet_data, size_t packet_len, uint8_t phase) {
    if (packet_len < 2) return std::nullopt;

    uint8_t length = packet_data[1];
    if (length != kExpectedDataLength) return std::nullopt;
    if (packet_len < kPhasePacketBytes) return std::nullopt;

    if (!xor_checksum_matches(packet_data, phase, length)) return std::nullopt;

    return unpack_le_int16_channels(packet_data + 2);
}

std::vector<std::array<int16_t, kMaxChannels>> ChannelsParser::process(const uint8_t* data,
                                                                       size_t length) {
    buffer_.insert(buffer_.end(), data, data + length);

    std::vector<std::array<int16_t, kMaxChannels>> results;

    while (buffer_.size() >= kSyncLen + 2) {
        const size_t sync_pos = find_sync_sequence(buffer_);

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
