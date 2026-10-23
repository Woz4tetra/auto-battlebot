#pragma once
#include <Arduino.h>
#include <driver/rmt.h>
#include <freertos/ringbuf.h>

enum dshot_mode_t {
    DSHOT150,
    DSHOT300,
    DSHOT600,
};

// Bidirectional DShot: inverted frames out, and the ESC's eRPM reply read back on the same pin
// through an RX channel. BLHeli_32 sees the inverted signal and switches modes on its own.
class DShot
{
public:
    DShot(gpio_num_t pin, rmt_channel_t tx_channel, rmt_channel_t rx_channel,
          dshot_mode_t mode = DSHOT300);
    bool begin();
    // Collects the reply to the previous frame, arms the receiver, and sends this frame. The
    // reply arrives about 30 us after the frame, so it is read one call later and never waited on.
    void send(uint16_t throttle, bool telemetry = false);
    // send() in two steps, so several ESCs transmit together. Frames sent one after another put
    // one ESC's frame on the wire during its neighbour's reply, and the crosstalk corrupts it.
    void start(uint16_t throttle, bool telemetry = false);
    void finish();
    // The reply collected by the latest send(): false when there was none or it failed the GCR
    // or CRC check.
    bool read_erpm(uint32_t &erpm) const;
    // Replies that arrived but did not decode, from boot.
    uint32_t decode_errors() const { return _decode_errors; }

private:
    gpio_num_t _pin;
    rmt_channel_t _channel;
    rmt_channel_t _rx_channel;
    dshot_mode_t _mode;
    bool _installed;
    RingbufHandle_t _rx_ring;
    bool _rx_armed;
    bool _reply_ok;
    uint32_t _erpm;
    uint32_t _decode_errors;
    float _reply_bit_ticks;

    rmt_item32_t _items[17]; // 16 data bits + 1 end marker
    uint16_t _t1h_ticks;
    uint16_t _t1l_ticks;
    uint16_t _t0h_ticks;
    uint16_t _t0l_ticks;

    void encode_frame(uint16_t packet);
    void collect_reply();
    void set_output(bool enabled);
    static uint16_t build_packet(uint16_t throttle, bool telemetry);
};
