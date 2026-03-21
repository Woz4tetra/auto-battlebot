#include "blheli_passthrough.h"
#include "blheli_esc_serial.h"
#include "blheli_msp.h"
#include "blheli_4way.h"

static bool active = false;
static uint16_t rx_counter = 0;
static uint16_t buffer_len = 0;
static uint8_t rx_buf[5001];

void passthrough_begin(uint8_t *pins, uint8_t count)
{
    if (active)
        return;
    blheli_esc_serial_init(pins, count);
    rx_counter = 0;
    active = true;
}

void passthrough_end()
{
    if (!active)
        return;
    blheli_esc_serial_end();
    blheli_esc_serial_deinit();
    rx_counter = 0;
    active = false;
}

void passthrough_process()
{
    if (!active)
        return;

    while (Serial.available())
    {
        rx_buf[rx_counter] = Serial.read();
        if (rx_counter < 5000)
            rx_counter++;
    }

    if (rx_counter == 0)
        return;

    if (rx_buf[0] == 0x2F)
    {
        if (rx_buf[4] == 0)
            buffer_len = 256 + 7;
        else
            buffer_len = rx_buf[4] + 7;

        if (rx_counter >= buffer_len)
        {
            rx_counter = 0;
            uint16_t tx_len = Check_4Way(rx_buf);
            if (tx_len > 0)
            {
                Serial.write(rx_buf, tx_len);
                Serial.flush();
            }
        }
    }
    else if (rx_counter >= 6 && rx_buf[0] == 0x24 && rx_buf[1] == 0x4D && rx_buf[2] == 0x3C)
    {
        uint8_t data_len = rx_buf[3];
        uint16_t frame_len = data_len + 6;
        if (rx_counter >= frame_len)
        {
            uint8_t tx_len = MSP_Check(rx_buf, rx_counter);
            if (tx_len > 0)
            {
                Serial.write(rx_buf, tx_len);
                Serial.flush();
            }
            rx_counter = 0;
        }
    }
    else if (rx_buf[0] != 0x24 && rx_buf[0] != 0x2F)
    {
        rx_counter = 0;
    }
}

bool passthrough_is_active()
{
    return active;
}
