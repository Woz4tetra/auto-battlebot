#include "crsf/crsf_parser.hpp"

#include <cstring>

namespace auto_battlebot {

// CRC8 lookup table: polynomial x^8 + x^7 + x^6 + x^4 + x^2 + 1 (0xD5)
// clang-format off
static constexpr uint8_t kCrc8Table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};
// clang-format on

static constexpr uint8_t kStartBytes[] = {0x00, 0xEE, 0xEA};
static constexpr size_t kMinFrameLength = 2;
static constexpr size_t kMaxFrameLength = 64;

uint8_t crsf_crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = kCrc8Table[crc ^ data[i]];
    }
    return crc;
}

static bool is_start_byte(uint8_t b) {
    for (uint8_t s : kStartBytes) {
        if (b == s) return true;
    }
    return false;
}

CrsfParseResult CrsfParser::parse_single_packet(const uint8_t* frame, size_t frame_len) {
    // frame: [start][length][type][payload...][crc8]
    uint8_t received_crc = frame[frame_len - 1];
    // CRC covers [type][payload] = frame[2 .. frame_len-2]
    uint8_t calc_crc = crsf_crc8(frame + 2, frame_len - 3);
    if (received_crc != calc_crc) {
        return {std::nullopt, CrsfParseError{"CRC mismatch: expected " + std::to_string(calc_crc) +
                                             " got " + std::to_string(received_crc)}};
    }

    uint8_t frame_type = frame[2];
    const uint8_t* payload = frame + 3;
    size_t payload_len = frame_len - 4;  // exclude start, length, type, crc

    switch (static_cast<CrsfFrameType>(frame_type)) {
        case CrsfFrameType::BATTERY: {
            if (payload_len < 7) {
                return {std::nullopt, CrsfParseError{"Battery payload too short"}};
            }
            CrsfBattery pkt;
            pkt.voltage = static_cast<float>((uint16_t(payload[0]) << 8) | payload[1]) / 10.0f;
            pkt.current = static_cast<float>((uint16_t(payload[2]) << 8) | payload[3]) / 10.0f;
            pkt.consumption = static_cast<float>((uint32_t(payload[4]) << 16) |
                                                 (uint32_t(payload[5]) << 8) | payload[6]);
            return {pkt, std::nullopt};
        }
        case CrsfFrameType::LINK_STATISTICS: {
            if (payload_len < 10) {
                return {std::nullopt, CrsfParseError{"LinkStatistics payload too short"}};
            }
            CrsfLinkStatistics pkt;
            pkt.up_rssi_ant1 = payload[0];
            pkt.up_rssi_ant2 = payload[1];
            pkt.up_link_quality = payload[2];
            pkt.up_snr = static_cast<int8_t>(payload[3]);
            pkt.active_antenna = payload[4];
            pkt.rf_profile = payload[5];
            pkt.up_rf_power = payload[6];
            pkt.down_rssi = payload[7];
            pkt.down_link_quality = payload[8];
            pkt.down_snr = static_cast<int8_t>(payload[9]);
            return {pkt, std::nullopt};
        }
        case CrsfFrameType::ATTITUDE: {
            if (payload_len < 6) {
                return {std::nullopt, CrsfParseError{"Attitude payload too short"}};
            }
            CrsfAttitude pkt;
            pkt.roll =
                static_cast<float>(static_cast<int16_t>((uint16_t(payload[0]) << 8) | payload[1])) /
                10000.0f;
            pkt.pitch =
                static_cast<float>(static_cast<int16_t>((uint16_t(payload[2]) << 8) | payload[3])) /
                10000.0f;
            pkt.yaw =
                static_cast<float>(static_cast<int16_t>((uint16_t(payload[4]) << 8) | payload[5])) /
                10000.0f;
            return {pkt, std::nullopt};
        }
        case CrsfFrameType::FLIGHT_MODE: {
            if (payload_len == 0 || payload[payload_len - 1] != 0x00) {
                return {std::nullopt, CrsfParseError{"FlightMode string not null-terminated"}};
            }
            CrsfFlightMode pkt;
            pkt.flight_mode = std::string(reinterpret_cast<const char*>(payload), payload_len - 1);
            return {pkt, std::nullopt};
        }
        default:
            return {std::nullopt,
                    CrsfParseError{"Unsupported frame type: " + std::to_string(frame_type)}};
    }
}

std::vector<CrsfParseResult> CrsfParser::parse(const uint8_t* data, size_t length) {
    buffer_.insert(buffer_.end(), data, data + length);

    std::vector<CrsfParseResult> results;
    size_t pos = 0;

    while (pos + 4 <= buffer_.size()) {
        if (!is_start_byte(buffer_[pos])) {
            ++pos;
            continue;
        }
        uint8_t frame_length = buffer_[pos + 1];
        if (frame_length < kMinFrameLength || frame_length > kMaxFrameLength) {
            ++pos;
            continue;
        }
        size_t total_len = static_cast<size_t>(frame_length) + 2;
        if (pos + total_len > buffer_.size()) {
            break;
        }
        results.push_back(parse_single_packet(buffer_.data() + pos, total_len));
        pos += total_len;
    }

    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<ptrdiff_t>(pos));
    return results;
}

std::vector<CrsfParseResult> CrsfParser::parse(const std::vector<uint8_t>& data) {
    return parse(data.data(), data.size());
}

}  // namespace auto_battlebot
