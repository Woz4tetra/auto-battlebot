#pragma once
#include <Arduino.h>
#include <DShotRMT.h>

namespace esc
{
    // DShot 3D mode throttle ranges
    const uint16_t DSHOT_STOP = 0;
    const uint16_t DSHOT_3D_REV_FULL = 48;
    const uint16_t DSHOT_3D_REV_SLOW = 1047;
    const uint16_t DSHOT_3D_FWD_SLOW = 1048;
    const uint16_t DSHOT_3D_FWD_FULL = 2047;

    class Esc
    {
    private:
        gpio_num_t pin;
        rmt_channel_t rmt_channel;
        DShotRMT *motor;
        bool initialized;
        uint16_t percent_to_dshot_3d(float signed_percent);

    public:
        Esc(gpio_num_t pin, rmt_channel_t rmt_channel);
        void begin();
        void deinit();
        void stop();
        void write(float signed_percent);
    };

}
