#pragma once

// Robot-side acceptance rules for ESP-NOW commands. No Arduino dependency so the native tests
// cover it. See "Rules the implementation must follow" in docs/plans/espnow_link_plan.md.

#include <stdint.h>

#include "espnow_protocol.h"

namespace espnow_protocol {

// 15 missed packets at the dongle's 250 Hz repeat rate. Kept generous on purpose: falling back to
// the trainer path costs ~5 ms, so a window that drops out on transient loss works against the
// latency goal. Tighten only if stale commands prove to be a real hazard.
const uint32_t COMMAND_STALE_MS = 60;

// 3, not 5: at 250 Hz that is a 12 ms recovery instead of 20 ms. Re-enter the fast path sooner and
// accept slightly more switching at the edge of range, which the firmware slew limiter absorbs.
const uint8_t FRESH_STREAK_REQUIRED = 3;

// Well above COMMAND_STALE_MS. Silence this long re-baselines the sequence number (an app restart
// starts again from 0) and, when the channel is not pinned, starts the robot's channel sweep.
const uint32_t REACQUIRE_SILENCE_MS = 500;

class CommandTracker {
   public:
    enum Result { ACCEPTED_NEW, ACCEPTED_REPEAT, DROPPED_OLD };

    /**
     * Applies the sequence rules to one received command:
     *   seq >  last: a new command; values and arrival time update.
     *   seq == last: a dongle repeat; arrival time updates only.
     *   seq <  last: a late frame from before the current command; dropped, unless
     *                REACQUIRE_SILENCE_MS has passed, in which case it is a restarted sender and
     *                becomes the new baseline.
     */
    Result on_command(const CommandPayload &command, uint32_t now_ms);

    /** The current command, new or repeated, arrived within COMMAND_STALE_MS, and at least
     *  FRESH_STREAK_REQUIRED frames have arrived without a stale gap between them. Leaving is
     *  immediate; re-entering takes a fresh streak. */
    bool fresh(uint32_t now_ms) const;

    const CommandPayload &latest() const { return latest_; }
    bool has_command() const { return has_command_; }
    /** Since the last accepted frame. UINT32_MAX before the first. */
    uint32_t age_ms(uint32_t now_ms) const;

   private:
    CommandPayload latest_ = {0, 0, 0.0f, 0.0f};
    bool has_command_ = false;
    uint32_t last_rx_ms_ = 0;
    uint8_t streak_ = 0;
};

/**
 * Go-back-N receive side of the OTA transfer. Tracks the next expected byte offset; a chunk is
 * accepted only when it starts exactly there. The caller writes accepted chunks to flash in order.
 */
class OtaSequencer {
   public:
    // Status after this many accepted chunks, so the host's 16-chunk window keeps moving.
    static const uint32_t ACK_EVERY = 8;

    void begin(uint32_t size);
    void reset();

    enum Result { ACCEPT, DROP };
    /** ACCEPT advances next_offset() by length. A chunk that starts elsewhere, is empty, or runs
     *  past the image size is a DROP. */
    Result on_chunk(uint32_t offset, uint32_t length);

    /** True once after every ACK_EVERY accepted chunks, and after the final chunk. */
    bool take_ack_due();

    uint32_t next_offset() const { return next_offset_; }
    uint32_t size() const { return size_; }
    bool complete() const { return active_ && next_offset_ == size_; }
    bool active() const { return active_; }

   private:
    uint32_t size_ = 0;
    uint32_t next_offset_ = 0;
    uint32_t accepted_since_ack_ = 0;
    bool ack_due_ = false;
    bool active_ = false;
};

}  // namespace espnow_protocol
