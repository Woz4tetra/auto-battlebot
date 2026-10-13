#include "espnow_protocol.h"

#include <string.h>

namespace espnow_protocol {

uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

size_t encode_serial_frame(uint8_t type, const uint8_t *payload, uint8_t length, uint8_t *out) {
    out[0] = SERIAL_MAGIC & 0xFF;
    out[1] = SERIAL_MAGIC >> 8;
    out[2] = type;
    out[3] = length;
    if (length > 0) memcpy(out + 4, payload, length);
    const uint16_t crc = crc16_ccitt(out + 2, 2 + length);
    out[4 + length] = crc & 0xFF;
    out[5 + length] = crc >> 8;
    return SERIAL_OVERHEAD + length;
}

size_t encode_air_frame(uint8_t type, const uint8_t *payload, size_t length, uint8_t *out) {
    if (length > AIR_MAX_PAYLOAD) return 0;
    memcpy(out, AIR_MAGIC, sizeof(AIR_MAGIC));
    out[sizeof(AIR_MAGIC)] = type;
    if (length > 0) memcpy(out + AIR_HEADER_SIZE, payload, length);
    return AIR_HEADER_SIZE + length;
}

bool is_air_frame(const uint8_t *frame, size_t length) {
    return length >= AIR_HEADER_SIZE && length <= AIR_MAX_FRAME &&
           memcmp(frame, AIR_MAGIC, sizeof(AIR_MAGIC)) == 0;
}

void SerialFrameParser::feed(const uint8_t *data, size_t count, Handler handler, void *context) {
    const uint8_t magic_lo = SERIAL_MAGIC & 0xFF;
    const uint8_t magic_hi = SERIAL_MAGIC >> 8;
    for (size_t i = 0; i < count; i++) {
        // A frame resolves the moment its last byte arrives, so the buffer never holds more than
        // one maximum-size frame.
        buffer_[size_++] = data[i];
        while (size_ > 0) {
            if (buffer_[0] != magic_lo) {
                drop(1);
                continue;
            }
            if (size_ < 2) break;
            if (buffer_[1] != magic_hi) {
                drop(1);
                continue;
            }
            if (size_ < 4) break;
            const uint8_t length = buffer_[3];
            const size_t frame_size = SERIAL_OVERHEAD + length;
            if (size_ < frame_size) break;

            const uint16_t expected = crc16_ccitt(buffer_ + 2, 2 + length);
            const uint16_t received =
                static_cast<uint16_t>(buffer_[4 + length] | (buffer_[5 + length] << 8));
            if (expected == received) {
                handler(context, buffer_[2], buffer_ + 4, length);
                drop(frame_size);
            } else {
                crc_errors_++;
                drop(1);
            }
        }
    }
}

void SerialFrameParser::drop(size_t count) {
    memmove(buffer_, buffer_ + count, size_ - count);
    size_ -= count;
}

}  // namespace espnow_protocol
