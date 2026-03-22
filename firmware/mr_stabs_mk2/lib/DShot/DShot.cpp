#include "DShot.h"

// Declared by Adafruit NeoPixel esp.c on IDF <5.
// Mark our channels so NeoPixel skips them.
extern bool rmt_reserved_channels[];

// DShot bit timing in nanoseconds
//   Bit‑1: 75% high, 25% low
//   Bit‑0: 37.5% high, 62.5% low
static const uint32_t DSHOT_BIT_NS[] = {
    6670, // DSHOT150
    3333, // DSHOT300
    1667, // DSHOT600
};

DShot::DShot(gpio_num_t pin, rmt_channel_t channel, dshot_mode_t mode)
    : _pin(pin), _channel(channel), _mode(mode), _installed(false),
      _t1h_ticks(0), _t1l_ticks(0), _t0h_ticks(0), _t0l_ticks(0)
{
    memset(_items, 0, sizeof(_items));
}

bool DShot::begin()
{
    if (_installed)
        return true;

    rmt_reserved_channels[_channel] = true;

    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = _channel;
    config.gpio_num = _pin;
    config.clk_div = 2; // 80 MHz APB / 2 = 40 MHz => 25 ns/tick
    config.mem_block_num = 1;
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    esp_err_t err = rmt_config(&config);
    if (err != ESP_OK)
        return false;

    err = rmt_driver_install(_channel, 0, 0);
    if (err != ESP_OK)
        return false;

    // Compute tick counts at 40 MHz (25 ns/tick)
    uint32_t bit_ns = DSHOT_BIT_NS[_mode];
    uint32_t total_ticks = bit_ns / 25;
    _t1h_ticks = (total_ticks * 3) / 4;      // 75%
    _t1l_ticks = total_ticks - _t1h_ticks;
    _t0h_ticks = (total_ticks * 3) / 8;      // 37.5%
    _t0l_ticks = total_ticks - _t0h_ticks;

    _installed = true;
    return true;
}

uint16_t DShot::build_packet(uint16_t throttle, bool telemetry)
{
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

    // CRC: XOR of three nibbles of the 12-bit value
    uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (packet << 4) | crc;
}

void DShot::encode_frame(uint16_t packet)
{
    for (int i = 0; i < 16; i++)
    {
        bool bit = (packet >> (15 - i)) & 1;
        _items[i].level0 = 1;
        _items[i].duration0 = bit ? _t1h_ticks : _t0h_ticks;
        _items[i].level1 = 0;
        _items[i].duration1 = bit ? _t1l_ticks : _t0l_ticks;
    }
    // End marker
    _items[16].level0 = 0;
    _items[16].duration0 = 0;
    _items[16].level1 = 0;
    _items[16].duration1 = 0;
}

void DShot::send(uint16_t throttle, bool telemetry)
{
    if (!_installed)
        return;

    uint16_t packet = build_packet(throttle, telemetry);
    encode_frame(packet);
    rmt_write_items(_channel, _items, 17, true);
}
