#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "espnow_protocol.h"

namespace espnow_protocol {

struct TelemetryBatchQuery {
    bool has_from = false;
    uint32_t from = 0;
    uint32_t max = TELEMETRY_BATCH_MAX;
};

inline uint32_t telemetry_batch_record_limit(const TelemetryBatchQuery &query) {
    return query.has_from ? query.max : 0;
}

inline size_t telemetry_batch_max_bytes(const TelemetryBatchQuery &query) {
    return sizeof(TelemetryBatchHeader) +
           telemetry_batch_record_limit(query) * sizeof(TelemetryPayload);
}

template <typename Ring>
size_t encode_telemetry_batch(const Ring &ring, uint32_t boot_id, const TelemetryBatchQuery &query,
                              uint8_t *out) {
    TelemetryPayload *records =
        reinterpret_cast<TelemetryPayload *>(out + sizeof(TelemetryBatchHeader));
    const auto result = ring.read(query.has_from ? query.from : ring.head(), records,
                                  telemetry_batch_record_limit(query));
    TelemetryBatchHeader header = {};
    header.magic = TELEMETRY_BATCH_MAGIC;
    header.version = TELEMETRY_BATCH_VERSION;
    header.record_size = sizeof(TelemetryPayload);
    header.boot_id = boot_id;
    header.capacity = ring.capacity();
    header.next_seq = result.next_seq;
    header.oldest_seq = result.oldest_seq_at_start;
    header.first_seq = result.first_seq;
    header.count = result.copied;
    std::memcpy(out, &header, sizeof(header));
    return sizeof(TelemetryBatchHeader) + result.copied * sizeof(TelemetryPayload);
}

}  // namespace espnow_protocol
