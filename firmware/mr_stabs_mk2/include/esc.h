#pragma once
#include <Arduino.h>
#include <DShot.h>
#include <esc_map.h>

namespace esc {
const uint32_t ERPM_VALID_MS = 50;

class Esc {
   private:
    gpio_num_t pin;
    rmt_channel_t channel;
    rmt_channel_t rx_channel;
    DShot *motor;
    bool initialized;
    uint32_t last_erpm = 0;
    uint32_t last_erpm_ms = 0;
    bool have_erpm = false;
    void take_erpm();
    void stop();
    static void send_pair(Esc &left, uint16_t left_value, Esc &right, uint16_t right_value);

   public:
    float stop_threshold = 1.0f;
    Esc(gpio_num_t pin, rmt_channel_t channel, rmt_channel_t rx_channel);
    bool begin();
    void deinit();
    static void write_pair(Esc &left, float left_percent, Esc &right, float right_percent);
    static void stop_pair(Esc &left, Esc &right);
    float duty(float signed_percent) const;
    uint32_t erpm() const { return last_erpm; }
    bool erpm_valid() const;
    uint32_t reply_decode_errors() const;
};

}  // namespace esc
