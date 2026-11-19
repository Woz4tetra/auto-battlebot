#pragma once

#include <stddef.h>

namespace usb_framing {
inline size_t copy_drainable_log(const char *ring, size_t capacity, size_t read_index,
                                 size_t write_index, char *out, size_t &copied) {
    copied = 0;
    size_t last_line = 0;
    for (size_t i = read_index; i != write_index; i = (i + 1) % capacity) {
        out[copied++] = ring[i];
        if (ring[i] == '\n') last_line = copied;
    }
    const bool ring_full_without_newline = last_line == 0 && copied >= capacity - 1;
    return ring_full_without_newline ? copied : last_line;
}
}  // namespace usb_framing
