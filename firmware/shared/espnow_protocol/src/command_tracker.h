#pragma once

#include <stdint.h>

#include "espnow_protocol.h"

namespace espnow_protocol {

const uint32_t COMMAND_STALE_MS = 60;

const uint8_t FRESH_STREAK_REQUIRED = 3;

const uint32_t REACQUIRE_SILENCE_MS = 500;

class CommandTracker {
   public:
    enum Result { ACCEPTED_NEW, ACCEPTED_REPEAT, DROPPED_OLD };

    Result on_command(const CommandPayload &command, uint32_t now_ms);

    bool fresh(uint32_t now_ms) const;

    const CommandPayload &latest() const { return latest_; }
    bool has_command() const { return has_command_; }
    uint32_t age_ms(uint32_t now_ms) const;

   private:
    CommandPayload latest_ = {0, 0, 0.0f, 0.0f};
    bool has_command_ = false;
    uint32_t last_rx_ms_ = 0;
    uint8_t streak_ = 0;
};

class OtaSequencer {
   public:
    // Status after this many accepted chunks, so the host's 16-chunk window keeps moving.
    static const uint32_t ACK_EVERY = 8;

    void begin(uint32_t size);
    void reset();

    enum Result { ACCEPT, DROP };
    Result on_chunk(uint32_t offset, uint32_t length);

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
