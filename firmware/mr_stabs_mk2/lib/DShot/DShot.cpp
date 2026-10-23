#include "DShot.h"

#include <erpm_decode.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>

// Declared by Adafruit NeoPixel esp.c on IDF <5.
// Mark our channels so NeoPixel skips them.
extern bool rmt_reserved_channels[];

// DShot bit timing in nanoseconds
//   Bit‑1: 75% active, 25% idle
//   Bit‑0: 37.5% active, 62.5% idle
// Inverted for bidirectional DShot: the line idles high and each bit starts low.
static const uint32_t DSHOT_BIT_NS[] = {
    6670,  // DSHOT150
    3333,  // DSHOT300
    1667,  // DSHOT600
};

static const uint32_t TICK_NS = 25;  // 80 MHz APB / 2
// The S3's RX-only channels are 4 to 7; their matrix signals count from RMT_SIG_IN0_IDX.
static const int RMT_FIRST_RX_CHANNEL = 4;
// The receiver arms before the frame and records frame, gap, and reply as one reception, so it
// can never arm too late for the reply. The idle threshold must outlast the gap (about 30 us);
// 80 us of quiet ends the reception.
static const uint16_t RX_IDLE_TICKS = 80000 / TICK_NS;
// A high run this long is the gap between our frame and the reply. The frame's longest high run
// is 2.1 us and the reply's is 8 us at DShot300.
static const uint32_t GAP_TICKS = 12000 / TICK_NS;
// Glitch filter in APB cycles: 0.5 us, a fifth of a reply bit at DShot300.
static const uint8_t RX_FILTER_APB_CYCLES = 40;
static const size_t RX_RING_BYTES = 1024;
static const size_t START_TRIES = 3;
static const size_t MAX_RUNS = 96;  // the RX memory block's 48 items

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
    config.clk_div = 2;  // 80 MHz APB / 2 = 40 MHz => 25 ns/tick
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
    rx.clk_div = 2;
    rx.mem_block_num = 1;
    rx.rx_config.filter_en = true;
    rx.rx_config.filter_ticks_thresh = RX_FILTER_APB_CYCLES;
    rx.rx_config.idle_threshold = RX_IDLE_TICKS;

    err = rmt_config(&rx);
    if (err != ESP_OK) return false;
    err = rmt_driver_install(_rx_channel, RX_RING_BYTES, 0);
    if (err != ESP_OK) return false;
    rmt_get_ringbuf_handle(_rx_channel, &_rx_ring);

    // rmt_config() pointed the pin at one channel or the other. Connect both: the TX channel's
    // output and the RX channel's input on the same pad, with the output enable taken from the
    // GPIO register so send() can release the line for the reply. The pull-up holds it high then.
    gpio_set_pull_mode(_pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(_pin, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(_pin, RMT_SIG_OUT0_IDX + _channel, false, false);
    esp_rom_gpio_connect_in_signal(_pin, RMT_SIG_IN0_IDX + (_rx_channel - RMT_FIRST_RX_CHANNEL),
                                   false);
    REG_SET_BIT(GPIO_FUNC0_OUT_SEL_CFG_REG + _pin * 4, GPIO_FUNC0_OEN_SEL);

    // Compute tick counts at 40 MHz (25 ns/tick)
    uint32_t bit_ns = DSHOT_BIT_NS[_mode];
    uint32_t total_ticks = bit_ns / TICK_NS;
    _t1h_ticks = (total_ticks * 3) / 4;  // 75%
    _t1l_ticks = total_ticks - _t1h_ticks;
    _t0h_ticks = (total_ticks * 3) / 8;  // 37.5%
    _t0l_ticks = total_ticks - _t0h_ticks;
    // The reply runs at 5/4 of the frame's bit rate.
    _reply_bit_ticks = bit_ns * 4.0f / 5.0f / TICK_NS;

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
    // Clears the matrix routing begin() set up, the output enable select included.
    gpio_reset_pin(_pin);
    _installed = false;
}

uint16_t DShot::build_packet(uint16_t throttle, bool telemetry) {
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

    // CRC: XOR of three nibbles of the 12-bit value, inverted for bidirectional DShot
    uint16_t crc = (~(packet ^ (packet >> 4) ^ (packet >> 8))) & 0x0F;
    return (packet << 4) | crc;
}

void DShot::encode_frame(uint16_t packet) {
    for (int i = 0; i < 16; i++) {
        bool bit = (packet >> (15 - i)) & 1;
        _items[i].level0 = 0;
        _items[i].duration0 = bit ? _t1h_ticks : _t0h_ticks;
        _items[i].level1 = 1;
        _items[i].duration1 = bit ? _t1l_ticks : _t0l_ticks;
    }
    // End marker
    _items[16].level0 = 1;
    _items[16].duration0 = 0;
    _items[16].level1 = 1;
    _items[16].duration1 = 0;
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

    // Drain the ring and keep the first reception that decodes. The driver re-arms the receiver
    // after every reception, so anything after the reply is noise picked up while the line sits
    // on the pull-up for the rest of the loop (worst with WiFi streaming every loop).
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
        // The final high run merges into the idle line and the RMT records it as length 0.
        // Leave a 0 for the decoder to infer.
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

void DShot::send(uint16_t throttle, bool telemetry) {
    start(throttle, telemetry);
    finish();
}

void DShot::start(uint16_t throttle, bool telemetry) {
    if (!_installed) return;

    collect_reply();

    uint16_t packet = build_packet(throttle, telemetry);
    encode_frame(packet);
    // Armed before the frame, so no software latency can miss the reply's start. The capture
    // holds our frame, the gap, and the reply; collect_reply() skips to the gap.
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
