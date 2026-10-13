#include "command_tracker.h"

namespace espnow_protocol {

CommandTracker::Result CommandTracker::on_command(const CommandPayload &command, uint32_t now_ms) {
    const uint32_t silence_ms = now_ms - last_rx_ms_;
    Result result;
    if (!has_command_ || command.seq > latest_.seq) {
        result = ACCEPTED_NEW;
    } else if (command.seq == latest_.seq) {
        result = ACCEPTED_REPEAT;
    } else if (silence_ms > REACQUIRE_SILENCE_MS) {
        result = ACCEPTED_NEW;
    } else {
        return DROPPED_OLD;
    }

    // A gap longer than the stale window breaks the streak, so re-entry always takes
    // FRESH_STREAK_REQUIRED frames in a row.
    if (!has_command_ || silence_ms > COMMAND_STALE_MS) {
        streak_ = 1;
    } else if (streak_ < FRESH_STREAK_REQUIRED) {
        streak_++;
    }

    if (result == ACCEPTED_NEW) latest_ = command;
    has_command_ = true;
    last_rx_ms_ = now_ms;
    return result;
}

bool CommandTracker::fresh(uint32_t now_ms) const {
    return has_command_ && now_ms - last_rx_ms_ <= COMMAND_STALE_MS &&
           streak_ >= FRESH_STREAK_REQUIRED;
}

uint32_t CommandTracker::age_ms(uint32_t now_ms) const {
    return has_command_ ? now_ms - last_rx_ms_ : UINT32_MAX;
}

void OtaSequencer::begin(uint32_t size) {
    size_ = size;
    next_offset_ = 0;
    accepted_since_ack_ = 0;
    ack_due_ = false;
    active_ = true;
}

void OtaSequencer::reset() {
    active_ = false;
    size_ = 0;
    next_offset_ = 0;
    accepted_since_ack_ = 0;
    ack_due_ = false;
}

OtaSequencer::Result OtaSequencer::on_chunk(uint32_t offset, uint32_t length) {
    if (!active_ || length == 0 || offset != next_offset_ || length > size_ - next_offset_) {
        return DROP;
    }
    next_offset_ += length;
    if (++accepted_since_ack_ >= ACK_EVERY || next_offset_ == size_) {
        accepted_since_ack_ = 0;
        ack_due_ = true;
    }
    return ACCEPT;
}

bool OtaSequencer::take_ack_due() {
    const bool due = ack_due_;
    ack_due_ = false;
    return due;
}

}  // namespace espnow_protocol
