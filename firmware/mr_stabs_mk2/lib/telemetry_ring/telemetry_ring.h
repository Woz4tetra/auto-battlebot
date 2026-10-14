#pragma once
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>

namespace telemetry_ring {

// What read() copied. Sequence numbers count every record ever pushed since boot, so a reader
// that asks for `from` and gets first_seq > from lost the records in between to overwrite.
struct read_result_t {
    uint32_t first_seq;  // seq of out[0]
    uint32_t count;      // records copied
    uint32_t head;       // seq the next push gets; head - 1 is the newest record
    uint32_t oldest;     // oldest seq still in the ring when the read started
};

// Fixed-size ring of records, one writer and any number of readers on other tasks or cores.
// Neither side locks: the control loop pushes without ever waiting on a reader, and a reader
// throws away whatever the writer overwrote while it was copying, the way a seqlock does.
//
// Record must be trivially copyable and have a uint32_t telemetry_seq field, which push() sets.
template <typename Record>
class TelemetryRing {
   public:
    // storage holds capacity records and outlives the ring. Capacity 0 makes every push a no-op.
    void init(Record *storage, uint32_t capacity) {
        storage_ = storage;
        capacity_ = capacity;
        head_.store(0, std::memory_order_relaxed);
        started_.store(0, std::memory_order_relaxed);
    }

    uint32_t capacity() const { return capacity_; }
    uint32_t head() const { return head_.load(std::memory_order_acquire); }

    // Stamps record.telemetry_seq and stores it. Writer side only.
    void push(Record record) {
        if (capacity_ == 0) return;
        const uint32_t seq = head_.load(std::memory_order_relaxed);
        record.telemetry_seq = seq;
        // Announce the slot before touching it, so a reader copying the old record there can
        // tell afterwards that it may be torn.
        started_.store(seq + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        std::memcpy(&storage_[seq % capacity_], &record, sizeof(Record));
        head_.store(seq + 1, std::memory_order_release);
    }

    // Copies up to max records starting at seq `from` into out, oldest first. A `from` older
    // than the ring holds starts at the oldest record; one at or past head copies nothing.
    read_result_t read(uint32_t from, Record *out, uint32_t max) const {
        const uint32_t head = head_.load(std::memory_order_acquire);
        const uint32_t oldest = head > capacity_ ? head - capacity_ : 0;
        uint32_t first = std::min(std::max(from, oldest), head);
        uint32_t count = std::min(head - first, max);
        for (uint32_t i = 0; i < count; i++) {
            std::memcpy(&out[i], &storage_[(first + i) % capacity_], sizeof(Record));
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        // Every slot the writer has started on since the copy began held a record older than
        // started - capacity. Those copies may be half old, half new: drop them.
        const uint32_t started = started_.load(std::memory_order_relaxed);
        const uint32_t valid = started > capacity_ ? started - capacity_ : 0;
        if (first < valid) {
            const uint32_t skip = std::min(valid - first, count);
            std::memmove(out, out + skip, (count - skip) * sizeof(Record));
            first += skip;
            count -= skip;
        }
        return {first, count, head, oldest};
    }

   private:
    Record *storage_ = nullptr;
    uint32_t capacity_ = 0;
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> started_{0};
};

}  // namespace telemetry_ring
