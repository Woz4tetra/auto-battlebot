#pragma once
#include <Arduino.h>
#include <DShotRMT.h>

namespace esc
{
    // DShot 3D mode: higher value = faster in both directions
    const uint16_t DSHOT_STOP = 0;
    const uint16_t DSHOT_3D_REV_MIN = 48;
    const uint16_t DSHOT_3D_REV_MAX = 1047;
    const uint16_t DSHOT_3D_FWD_MIN = 1048;
    const uint16_t DSHOT_3D_FWD_MAX = 2047;

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
        bool begin();
        void deinit();
        void stop();
        void write(float signed_percent);
    };

}
