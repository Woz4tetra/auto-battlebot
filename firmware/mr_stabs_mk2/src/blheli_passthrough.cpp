#include "blheli_passthrough.h"
#include "blheli_ble_comm.h"
#include "blheli_esc_serial.h"

static bool active = false;

void passthrough_begin(uint8_t *pins, uint8_t count)
{
    if (active)
        return;
    blheli_esc_serial_init(pins, count);
    blheli_ble_init();
    active = true;
    Serial.println("BLHeli passthrough started");
}

void passthrough_end()
{
    if (!active)
        return;
    blheli_esc_serial_deinit();
    blheli_ble_deinit();
    active = false;
    Serial.println("BLHeli passthrough stopped");
}

void passthrough_process()
{
    if (!active)
        return;
    blheli_ble_process();
}

bool passthrough_is_active()
{
    return active;
}
