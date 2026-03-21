#include <Arduino.h>
#include <SoftwareSerial.h>
#include "blheli_esc_serial.h"

bool Enable4Way = false;
uint8_t blheli_esc_pins[8] = {255, 255, 255, 255, 255, 255, 255, 255};
uint8_t blheli_esc_count = 0;
static uint8_t current_pin_index = 0;
static uint16_t esc_crc = 0;
static SoftwareSerial *swSer = nullptr;

void blheli_esc_serial_init(uint8_t *pins, uint8_t pin_count)
{
    if (pin_count > 8)
        pin_count = 8;
    blheli_esc_count = pin_count;
    for (uint8_t i = 0; i < pin_count; i++)
        blheli_esc_pins[i] = pins[i];
    current_pin_index = 0;
}

void blheli_esc_serial_deinit()
{
    blheli_esc_serial_end();
    blheli_esc_count = 0;
    for (uint8_t i = 0; i < 8; i++)
        blheli_esc_pins[i] = 255;
}

void blheli_esc_serial_switch_pin(uint8_t index)
{
    if (index >= blheli_esc_count)
        return;
    if (index == current_pin_index && Enable4Way)
        return;
    if (Enable4Way)
        blheli_esc_serial_end();
    current_pin_index = index;
    blheli_esc_serial_begin();
}

void blheli_esc_serial_begin()
{
    Enable4Way = true;
    uint8_t pin = blheli_esc_pins[current_pin_index];
    if (swSer)
    {
        swSer->end();
        delete swSer;
    }
    swSer = new SoftwareSerial();
    swSer->begin(19200, SWSERIAL_8N1, pin, pin, false, 512);
    swSer->enableIntTx(false);
}

void blheli_esc_serial_end()
{
    if (swSer)
    {
        swSer->end();
        delete swSer;
        swSer = nullptr;
    }
    Enable4Way = false;
}

uint16_t SendESC(uint8_t tx_buf[], uint16_t buf_size)
{
    return SendESC(tx_buf, buf_size, true);
}

uint16_t SendESC(uint8_t tx_buf[], uint16_t buf_size, bool CRC)
{
    if (!swSer)
        return 0;
    uint16_t i = 0;
    esc_crc = 0;
    if (buf_size == 0)
        buf_size = 256;

    swSer->enableTx(true);
    for (i = 0; i < buf_size; i++)
    {
        swSer->write(tx_buf[i]);
        esc_crc = ByteCrc(tx_buf[i], esc_crc);
    }
    if (CRC)
    {
        swSer->write(esc_crc & 0xff);
        swSer->write((esc_crc >> 8) & 0xff);
        buf_size = buf_size + 2;
    }
    swSer->enableTx(false);
    return 0;
}

uint16_t GetESC(uint8_t rx_buf[], uint16_t wait_ms)
{
    if (!swSer)
        return 0;
    uint16_t i = 0;
    esc_crc = 0;
    bool timeout = false;

    while ((!swSer->available()) && (!timeout))
    {
        delay(1);
        i++;
        if (i >= wait_ms)
        {
            timeout = true;
            return 0;
        }
    }
    i = 0;
    while (swSer->available())
    {
        rx_buf[i] = swSer->read();
        i++;
        delayMicroseconds(100);
    }
    return i;
}

uint16_t ByteCrc(uint8_t data, uint16_t crc)
{
    uint8_t xb = data;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (((xb & 0x01) ^ (crc & 0x0001)) != 0)
        {
            crc = crc >> 1;
            crc = crc ^ 0xA001;
        }
        else
        {
            crc = crc >> 1;
        }
        xb = xb >> 1;
    }
    return crc;
}
