#pragma once
#include <Arduino.h>
#include <DShot.h>

namespace esc
{
    const uint16_t DSHOT_STOP = 0;
    // DShot 3D mode: higher value = faster in both directions
    const uint16_t DSHOT_3D_REV_MIN = 48;
    const uint16_t DSHOT_3D_REV_MAX = 1047;
    const uint16_t DSHOT_3D_FWD_MIN = 1048;
    const uint16_t DSHOT_3D_FWD_MAX = 2047;
    // eRPM older than this reads invalid, and the motor guard falls back to the shunt.
    const uint32_t ERPM_VALID_MS = 50;

    class Esc
    {
    private:
        gpio_num_t pin;
        rmt_channel_t channel;
        rmt_channel_t rx_channel;
        DShot *motor;
        bool initialized;
        uint32_t last_erpm = 0;
        uint32_t last_erpm_ms = 0;
        bool have_erpm = false;
        uint16_t percent_to_dshot_3d(float signed_percent);
        void take_erpm();
        static void send_pair(Esc &left, uint16_t left_value, Esc &right, uint16_t right_value);

    public:
        float stop_threshold = 1.0f;
        Esc(gpio_num_t pin, rmt_channel_t channel, rmt_channel_t rx_channel);
        bool begin();
        void deinit();
        void stop();
        void write(float signed_percent);
        // Both ESCs at once: the frames go out together, so neither lands on the other ESC's
        // eRPM reply. Use these, not write() and stop().
        static void write_pair(Esc &left, float left_percent, Esc &right, float right_percent);
        static void stop_pair(Esc &left, Esc &right);
        // Duty the ESC runs for a command: 0 inside stop_threshold, else percent / 100.
        float duty(float signed_percent) const;
        // Latest eRPM reply, unsigned. Valid while a reply decoded within ERPM_VALID_MS.
        uint32_t erpm() const { return last_erpm; }
        bool erpm_valid() const;
        // Replies that failed the GCR or CRC check, from boot.
        uint32_t crc_errors() const;
    };

}
