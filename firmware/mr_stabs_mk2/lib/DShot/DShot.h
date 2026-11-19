#pragma once
#include <Arduino.h>
#include <driver/rmt.h>
#include <freertos/ringbuf.h>

enum dshot_mode_t {
    DSHOT150,
    DSHOT300,
    DSHOT600,
};

static const int FRAME_BITS = 16;

class DShot {
   public:
    DShot(gpio_num_t pin, rmt_channel_t tx_channel, rmt_channel_t rx_channel,
          dshot_mode_t mode = DSHOT300);
    ~DShot();
    bool begin();
    void end();
    void start(uint16_t throttle, bool telemetry = false);
    void finish();
    bool read_erpm(uint32_t &erpm) const;
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

    rmt_item32_t _items[FRAME_BITS + 1];
    uint16_t _t1h_ticks;
    uint16_t _t1l_ticks;
    uint16_t _t0h_ticks;
    uint16_t _t0l_ticks;

    void encode_frame(uint16_t packet);
    void collect_reply();
    void set_output(bool enabled);
};
