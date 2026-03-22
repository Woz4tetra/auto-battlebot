#pragma once
#include <Arduino.h>
#include <driver/rmt.h>

enum dshot_mode_t {
    DSHOT150,
    DSHOT300,
    DSHOT600,
};

class DShot
{
public:
    DShot(gpio_num_t pin, rmt_channel_t channel, dshot_mode_t mode = DSHOT300);
    bool begin();
    void send(uint16_t throttle, bool telemetry = false);

private:
    gpio_num_t _pin;
    rmt_channel_t _channel;
    dshot_mode_t _mode;
    bool _installed;

    rmt_item32_t _items[17]; // 16 data bits + 1 end marker
    uint16_t _t1h_ticks;
    uint16_t _t1l_ticks;
    uint16_t _t0h_ticks;
    uint16_t _t0l_ticks;

    void encode_frame(uint16_t packet);
    static uint16_t build_packet(uint16_t throttle, bool telemetry);
};
