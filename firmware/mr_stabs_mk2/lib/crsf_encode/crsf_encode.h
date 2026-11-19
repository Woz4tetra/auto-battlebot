#pragma once
#include <endian.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace crsf_encode {

inline uint16_t to_be_u16(float value) {
    if (!(value > 0.0f)) return 0;
    return htobe16(static_cast<uint16_t>(std::min(value, 65535.0f)));
}

inline int16_t to_be_i16(float value) {
    if (std::isnan(value)) return 0;
    return htobe16(static_cast<int16_t>(std::min(32767.0f, std::max(-32768.0f, value))));
}

}  // namespace crsf_encode
