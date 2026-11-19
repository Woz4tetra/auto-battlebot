#include "DShot.h"

#include <erpm_decode.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>

// Declared by Adafruit NeoPixel esp.c on IDF <5.
// Mark our channels so NeoPixel skips them.
extern bool rmt_reserved_channels[];

static uint32_t dshot_bit_ns(dshot_mode_t mode) {
    switch (mode) {
        case DSHOT150:
            return 6670;
        case DSHOT300:
            return 3333;
        case DSHOT600:
            return 1667;
    }
    return 3333;
}

static const uint32_t APB_HZ = 80000000;
static const uint32_t RMT_CLK_DIV = 2;
static const uint32_t TICK_NS = 1000000000 / (APB_HZ / RMT_CLK_DIV);
static const float REPLY_BIT_RATE_RATIO = 5.0f / 4.0f;
static const int RMT_FIRST_RX_CHANNEL = 4;
static const uint16_t RX_IDLE_TICKS = 80000 / TICK_NS;
static const uint32_t GAP_TICKS = 12000 / TICK_NS;
static const uint8_t RX_FILTER_APB_CYCLES_0_5US = 40;
static const size_t RX_RING_BYTES = 1024;
static const size_t START_TRIES = 3;
static const size_t MAX_RUNS = 96;

DShot::DShot(gpio_num_t pin, rmt_channel_t tx_channel, rmt_channel_t rx_channel, dshot_mode_t mode)
    : _pin(pin),
      _channel(tx_channel),
      _rx_channel(rx_channel),
      _mode(mode),
      _installed(false),
      _rx_ring(nullptr),
      _rx_armed(false),
      _reply_ok(false),
      _erpm(0),
      _decode_errors(0),
      _reply_bit_ticks(0.0f),
      _t1h_ticks(0),
      _t1l_ticks(0),
      _t0h_ticks(0),
      _t0l_ticks(0) {
    memset(_items, 0, sizeof(_items));
}

DShot::~DShot() { end(); }

bool DShot::begin() {
    if (_installed) return true;

    rmt_reserved_channels[_channel] = true;
    rmt_reserved_channels[_rx_channel] = true;

    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = _channel;
    config.gpio_num = _pin;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;

    esp_err_t err = rmt_config(&config);
    if (err != ESP_OK) return false;

    err = rmt_driver_install(_channel, 0, 0);
    if (err != ESP_OK) return false;

    rmt_config_t rx = {};
    rx.rmt_mode = RMT_MODE_RX;
    rx.channel = _rx_channel;
    rx.gpio_num = _pin;
    rx.clk_div = RMT_CLK_DIV;
    rx.mem_block_num = 1;
    rx.rx_config.filter_en = true;
    rx.rx_config.filter_ticks_thresh = RX_FILTER_APB_CYCLES_0_5US;
    rx.rx_config.idle_threshold = RX_IDLE_TICKS;

    err = rmt_config(&rx);
    if (err != ESP_OK) return false;
    err = rmt_driver_install(_rx_channel, RX_RING_BYTES, 0);
    if (err != ESP_OK) return false;
    rmt_get_ringbuf_handle(_rx_channel, &_rx_ring);

    gpio_set_pull_mode(_pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(_pin, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(_pin, RMT_SIG_OUT0_IDX + _channel, false, false);
    esp_rom_gpio_connect_in_signal(_pin, RMT_SIG_IN0_IDX + (_rx_channel - RMT_FIRST_RX_CHANNEL),
                                   false);
    REG_SET_BIT(GPIO_FUNC0_OUT_SEL_CFG_REG + _pin * 4, GPIO_FUNC0_OEN_SEL);

    uint32_t bit_ns = dshot_bit_ns(_mode);
    uint32_t total_ticks = bit_ns / TICK_NS;
    _t1h_ticks = (total_ticks * 3) / 4;
    _t1l_ticks = total_ticks - _t1h_ticks;
    _t0h_ticks = (total_ticks * 3) / 8;
    _t0l_ticks = total_ticks - _t0h_ticks;
    _reply_bit_ticks = bit_ns / REPLY_BIT_RATE_RATIO / TICK_NS;

    _installed = true;
    return true;
}

void DShot::end() {
    if (!_installed) return;
    if (_rx_armed) {
        rmt_rx_stop(_rx_channel);
        _rx_armed = false;
    }
    rmt_driver_uninstall(_channel);
    rmt_driver_uninstall(_rx_channel);
    _rx_ring = nullptr;
    _reply_ok = false;
    gpio_reset_pin(_pin);
    _installed = false;
}

void DShot::encode_frame(uint16_t packet) {
    for (int i = 0; i < FRAME_BITS; i++) {
        bool bit = (packet >> (15 - i)) & 1;
        _items[i].level0 = 0;
        _items[i].duration0 = bit ? _t1h_ticks : _t0h_ticks;
        _items[i].level1 = 1;
        _items[i].duration1 = bit ? _t1l_ticks : _t0l_ticks;
    }
    _items[FRAME_BITS].level0 = 1;
    _items[FRAME_BITS].duration0 = 0;
    _items[FRAME_BITS].level1 = 1;
    _items[FRAME_BITS].duration1 = 0;
}

void DShot::set_output(bool enabled) {
    const uint32_t bit = 1u << (_pin % 32);
    if (_pin < 32)
        REG_WRITE(enabled ? GPIO_ENABLE_W1TS_REG : GPIO_ENABLE_W1TC_REG, bit);
    else
        REG_WRITE(enabled ? GPIO_ENABLE1_W1TS_REG : GPIO_ENABLE1_W1TC_REG, bit);
}

void DShot::collect_reply() {
    _reply_ok = false;
    if (!_rx_armed) return;
    rmt_rx_stop(_rx_channel);
    _rx_armed = false;

    uint32_t runs[MAX_RUNS];
    bool received = false;
    size_t size = 0;
    rmt_item32_t *items;
    while ((items = (rmt_item32_t *)xRingbufferReceive(_rx_ring, &size, 0)) != nullptr) {
        received = true;
        size_t count = 0;
        const size_t n = size / sizeof(rmt_item32_t);
        const bool starts_low = n == 0 || items[0].level0 == 0;
        int last_level = 1;
        for (size_t i = 0; i < n && count < MAX_RUNS; i++) {
            if (items[i].duration0 == 0) break;
            runs[count++] = items[i].duration0;
            last_level = items[i].level0;
            if (items[i].duration1 == 0 || count >= MAX_RUNS) break;
            runs[count++] = items[i].duration1;
            last_level = items[i].level1;
        }
        if (last_level == 0 && count < MAX_RUNS) runs[count++] = 0;
        vRingbufferReturnItem(_rx_ring, items);
        // The reply starts on a low run, but the capture can open with a stray edge or two:
        // the neighbouring ESC's frame is often still on the wire when this receiver arms, and
        // a reception that began before our own frame holds it ahead of a long gap. Try the
        // first few low runs after any gap; the GCR and CRC reject a wrong start.
        size_t from = 0;
        for (size_t i = starts_low ? 1 : 0; i < count; i += 2) {
            if (runs[i] >= GAP_TICKS && i + 1 < count) {
                from = i + 1;
                break;
            }
        }
        if (from == 0 && !starts_low) from = 1;
        for (size_t first = from, tries = 0; !_reply_ok && first < count && tries < START_TRIES;
             first += 2, tries++) {
            uint32_t erpm = 0;
            if (erpm_decode::decode_measured(runs + first, count - first, _reply_bit_ticks, erpm)) {
                _erpm = erpm;
                _reply_ok = true;
            }
        }
    }
    if (received && !_reply_ok) _decode_errors++;
}

void DShot::start(uint16_t throttle, bool telemetry) {
    if (!_installed) return;

    collect_reply();

    uint16_t packet = erpm_decode::build_packet(throttle, telemetry);
    encode_frame(packet);
    rmt_rx_start(_rx_channel, true);
    _rx_armed = true;
    set_output(true);
    rmt_write_items(_channel, _items, 17, false);
}

void DShot::finish() {
    if (!_installed) return;
    rmt_wait_tx_done(_channel, portMAX_DELAY);
    set_output(false);
}

bool DShot::read_erpm(uint32_t &erpm) const {
    if (!_reply_ok) return false;
    erpm = _erpm;
    return true;
}
