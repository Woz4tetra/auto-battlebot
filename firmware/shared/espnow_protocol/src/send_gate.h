#pragma once

#include <stddef.h>
#include <stdint.h>

namespace espnow_protocol {

enum class SendPoll : uint8_t { READY, BUSY, CALLBACK_LOST };

class SendGate {
   public:
    explicit SendGate(uint32_t callback_timeout_ms) : timeout_ms_(callback_timeout_ms) {}

    SendPoll poll(uint32_t now_ms) {
        if (!in_flight_) return SendPoll::READY;
        if (now_ms - started_ms_ < timeout_ms_) return SendPoll::BUSY;
        in_flight_ = false;
        return SendPoll::CALLBACK_LOST;
    }

    void start(uint32_t now_ms) {
        started_ms_ = now_ms;
        in_flight_ = true;
    }

    void complete() { in_flight_ = false; }

   private:
    uint32_t timeout_ms_;
    volatile bool in_flight_ = false;
    uint32_t started_ms_ = 0;
};

enum class UsbTxAction : uint8_t { WRITE, RESET_STUCK_FIFO };

inline UsbTxAction usb_tx_action(int available_for_write, size_t frame_size) {
    return available_for_write >= static_cast<int>(frame_size) ? UsbTxAction::WRITE
                                                               : UsbTxAction::RESET_STUCK_FIFO;
}

}  // namespace espnow_protocol
