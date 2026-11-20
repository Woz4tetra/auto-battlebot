#pragma once

#include <stdint.h>

namespace bno055_decode {

constexpr uint8_t REG_GYRO_EULER = 0x14;
constexpr uint8_t REG_LINEAR_GRAVITY = 0x28;
constexpr uint8_t BLOCK_BYTES = 12;
constexpr uint8_t VECTOR_BYTES = 6;
constexpr double LSB_PER_DPS = 16.0;
constexpr double LSB_PER_DEGREE = 16.0;
constexpr double LSB_PER_MPS2 = 100.0;
constexpr float DPS_TO_RAD_S = 0.017453293f;

struct Vec3 {
    float x;
    float y;
    float z;
};

inline int16_t le16(const uint8_t *bytes) {
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                (static_cast<uint16_t>(bytes[1]) << 8));
}

inline Vec3 scaled(const uint8_t *bytes, double lsb_per_unit, double multiplier) {
    return {static_cast<float>(le16(bytes) / lsb_per_unit * multiplier),
            static_cast<float>(le16(bytes + 2) / lsb_per_unit * multiplier),
            static_cast<float>(le16(bytes + 4) / lsb_per_unit * multiplier)};
}

inline Vec3 gyro_rad_s(const uint8_t *gyro_euler_block) {
    return scaled(gyro_euler_block, LSB_PER_DPS, DPS_TO_RAD_S);
}

inline Vec3 euler_deg(const uint8_t *gyro_euler_block) {
    return scaled(gyro_euler_block + VECTOR_BYTES, LSB_PER_DEGREE, 1.0);
}

inline Vec3 linear_mps2(const uint8_t *linear_gravity_block) {
    return scaled(linear_gravity_block, LSB_PER_MPS2, 1.0);
}

inline Vec3 gravity_mps2(const uint8_t *linear_gravity_block) {
    return scaled(linear_gravity_block + VECTOR_BYTES, LSB_PER_MPS2, 1.0);
}

}  // namespace bno055_decode
