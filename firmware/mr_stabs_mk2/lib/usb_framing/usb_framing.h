#pragma once

#include <stddef.h>
#include <stdint.h>

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

inline bool push_log_char(char *ring, size_t capacity, size_t &write_index, size_t read_index,
                          char c) {
    const size_t next = (write_index + 1) % capacity;
    if (next == read_index) return false;
    ring[write_index] = c;
    write_index = next;
    return true;
}

template <typename Port, typename Now, typename Yield>
bool write_whole(Port &port, const uint8_t *data, size_t size, uint32_t timeout_ms, Now now,
                 Yield yield) {
    const uint32_t start = now();
    while (static_cast<size_t>(port.availableForWrite()) < size) {
        if (now() - start > timeout_ms) return false;
        yield();
    }
    port.write(data, size);
    return true;
}
}  // namespace usb_framing
