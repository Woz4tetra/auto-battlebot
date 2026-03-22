#include <esc.h>

using namespace esc;

Esc::Esc(gpio_num_t pin, rmt_channel_t channel)
    : pin(pin), channel(channel), motor(nullptr), initialized(false)
{
}

bool Esc::begin()
{
    if (initialized)
        return true;
    motor = new DShot(pin, channel, DSHOT300);
    if (!motor->begin())
    {
        delete motor;
        motor = nullptr;
        return false;
    }
    initialized = true;
    return true;
}

void Esc::deinit()
{
    if (!initialized)
        return;
    stop();
    delete motor;
    motor = nullptr;
    initialized = false;
}

void Esc::write(float signed_percent)
{
    if (!initialized)
        return;
    motor->send(percent_to_dshot_3d(signed_percent));
}

void Esc::stop()
{
    if (!initialized)
        return;
    motor->send(DSHOT_STOP);
}

uint16_t Esc::percent_to_dshot_3d(float signed_percent)
{
    signed_percent = constrain(signed_percent, -100.0f, 100.0f);
    if (signed_percent == 0.0f)
        return DSHOT_STOP;

    if (signed_percent > 0.0f)
    {
        return DSHOT_3D_FWD_MIN +
               (uint16_t)(signed_percent / 100.0f * (DSHOT_3D_FWD_MAX - DSHOT_3D_FWD_MIN));
    }
    else
    {
        float magnitude = -signed_percent / 100.0f;
        return DSHOT_3D_REV_MIN +
               (uint16_t)(magnitude * (DSHOT_3D_REV_MAX - DSHOT_3D_REV_MIN));
    }
}
