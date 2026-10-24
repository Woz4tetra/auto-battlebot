#include <driver/gpio.h>
#include <driver/uart.h>
#include <esc_config.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/uart_periph.h>

namespace esc_config {

const char *state_name(State state) {
    switch (state) {
        case State::COMBAT:
            return "combat";
        case State::TUNING:
            return "tuning";
        case State::EXITING:
            return "exiting";
    }
    return "unknown";
}

// The bootloader link on the ESC signal wires: 19200 8N1, half duplex, one wire per ESC.
//
// The passthrough (firmware/BlHeli-Passthrough/ESP32/ESC_Serial.cpp) bit-bangs this with
// EspSoftwareSerial. Inside this firmware that lost bits: 40 of 72 reads on 2026-10-23 needed a
// retry (bad CRCs, a 0x30 ack read as 0xfe), so the link uses UART2 instead. The GPIO matrix routes
// the UART's TX and RX to the selected ESC's pin, set open drain with the pull-up, so the ESC can
// pull the line low to answer. The receiver hears every byte sent, and receive() drops that echo.
class EscConfig::SerialLink : public blheli_boot::EscLink {
   public:
    SerialLink(uint8_t pin0, uint8_t pin1) : _pins{pin0, pin1} {}

    uint8_t count() const override { return 2; }

    void select(uint8_t esc) override {
        if (!_installed) {
            uart_config_t config = {};
            config.baud_rate = 19200;
            config.data_bits = UART_DATA_8_BITS;
            config.parity = UART_PARITY_DISABLE;
            config.stop_bits = UART_STOP_BITS_1;
            config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            config.source_clk = UART_SCLK_APB;
            uart_driver_install(PORT, 1024, 0, 0, nullptr, 0);
            uart_param_config(PORT, &config);
            // Hand each byte over as it lands. The default 120-byte threshold holds a reply back
            // for 62 ms at a time, longer than the 10 ms gap that ends it.
            uart_set_rx_full_threshold(PORT, 1);
            _installed = true;
        }
        if (_current >= 0 && _current != esc) drive(_pins[_current], 1);
        _current = esc;
        const gpio_num_t pin = static_cast<gpio_num_t>(_pins[esc]);
        // uart_set_pin() leaves the shared pin input only. Open drain gives the UART the line
        // back, and since gpio_set_direction() reconnects the pin to plain GPIO output, the
        // UART's TX signal is routed to it again last.
        uart_set_pin(PORT, pin, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
        esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(PORT, SOC_UART_TX_PIN_IDX), false,
                                        false);
        uart_flush_input(PORT);
        _echo = 0;
    }

    void send(const uint8_t *data, size_t size) override {
        if (_current < 0) return;
        // Anything still in the receiver belongs to an earlier exchange.
        uart_flush_input(PORT);
        uart_write_bytes(PORT, reinterpret_cast<const char *>(data), size);
        _echo = size;
    }

    size_t receive(uint8_t *buf, size_t max, size_t expect, uint32_t first_ms) override {
        if (_current < 0) return 0;
        // Our own bytes come back first. 256 of them take 135 ms on the wire.
        while (_echo > 0) {
            uint8_t sink[64];
            const size_t want = _echo < sizeof(sink) ? _echo : sizeof(sink);
            const int got = uart_read_bytes(PORT, sink, want, pdMS_TO_TICKS(200));
            if (got <= 0) {
                _echo = 0;
                break;
            }
            _echo -= got;
        }
        int got = uart_read_bytes(PORT, buf, 1, pdMS_TO_TICKS(first_ms));
        if (got <= 0) return 0;
        size_t n = 1;
        while (n < max && (expect == 0 || n < expect)) {
            // A gap of 10 ms ends the reply, as in the passthrough.
            got = uart_read_bytes(PORT, buf + n, 1, pdMS_TO_TICKS(10));
            if (got <= 0) break;
            n += got;
        }
        return n;
    }

    void hold_low(uint32_t ms) override {
        if (_current < 0) return;
        const uint8_t pin = _pins[_current];
        drive(pin, 0);
        delay(ms);
        drive(pin, 1);
        _current = -1;  // the next select() hands the pin back to the UART
    }

    void delay_ms(uint32_t ms) override { delay(ms); }

    // Drives both signal lines high. A BLHeli ESC that has lost its signal enters the bootloader
    // only if the line reads high when it checks, so neither line is left floating on a pull-up.
    void idle_high() {
        _current = -1;
        drive(_pins[0], 1);
        drive(_pins[1], 1);
    }

    // Gives both pins back for DShot.
    void release() {
        _current = -1;
        if (_installed) {
            // Park the receiver on a constant high input before the driver goes.
            esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                           UART_PERIPH_SIGNAL(PORT, SOC_UART_RX_PIN_IDX), false);
            uart_driver_delete(PORT);
            _installed = false;
        }
        gpio_reset_pin(static_cast<gpio_num_t>(_pins[0]));
        gpio_reset_pin(static_cast<gpio_num_t>(_pins[1]));
    }

   private:
    static const uart_port_t PORT = UART_NUM_2;  // UART1 is the CRSF receiver

    // Takes the pin from the UART and drives it as a plain GPIO.
    static void drive(uint8_t pin, uint32_t level) {
        const gpio_num_t gpio = static_cast<gpio_num_t>(pin);
        esp_rom_gpio_connect_out_signal(gpio, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio, level);
    }

    uint8_t _pins[2];
    int8_t _current = -1;
    size_t _echo = 0;  // bytes sent that the receiver has not given back yet
    bool _installed = false;
};

EscConfig::EscConfig() { _mux = portMUX_INITIALIZER_UNLOCKED; }

void EscConfig::begin(esc::Esc *right, esc::Esc *left, uint8_t right_pin, uint8_t left_pin) {
    _escs[0] = right;
    _escs[1] = left;
    _link = new SerialLink(right_pin, left_pin);
    _server = new blheli_boot::Server(*_link);
}

void EscConfig::set_error(const char *error) {
    portENTER_CRITICAL(&_mux);
    _error = error;
    portEXIT_CRITICAL(&_mux);
}

void EscConfig::request(bool enter) {
    portENTER_CRITICAL(&_mux);
    _requested = enter ? 1 : 0;
    portEXIT_CRITICAL(&_mux);
}

bool EscConfig::submit(const uint8_t *frame, size_t size, uint32_t &id, const char *&reason) {
    if (size > blheli_boot::MAX_REQUEST) {
        reason = "frame too long";
        return false;
    }
    bool ok = false;
    portENTER_CRITICAL(&_mux);
    if (_state != State::TUNING) {
        reason = "not in tuning mode";
    } else if (_slot_state == Poll::PENDING) {
        reason = "a frame is already pending";
    } else {
        memcpy(_request, frame, size);
        _request_size = size;
        _id = _next_id++;
        id = _id;
        _slot_state = Poll::PENDING;
        ok = true;
    }
    portEXIT_CRITICAL(&_mux);
    return ok;
}

Poll EscConfig::poll(uint32_t &id, uint8_t *out, size_t &size) {
    portENTER_CRITICAL(&_mux);
    const Poll state = _slot_state;
    id = _id;
    size = 0;
    if (state == Poll::DONE) {
        memcpy(out, _response, _response_size);
        size = _response_size;
    }
    portEXIT_CRITICAL(&_mux);
    return state;
}

status_t EscConfig::status() {
    status_t s = {};
    s.state = _state;
    s.idle_ms = _state == State::TUNING ? millis() - _last_activity_ms : 0;
    s.released = _released;
    s.guard_refusals = _server ? _server->guard_refusals() : 0;
    portENTER_CRITICAL(&_mux);
    s.error = _error;
    s.frames = _frames;
    s.pending = _slot_state == Poll::PENDING;
    s.armed = _armed;
    s.driving = _driving;
    memcpy(s.failure, _failure, sizeof(s.failure));
    portEXIT_CRITICAL(&_mux);
    return s;
}

void EscConfig::enter(uint32_t now_ms) {
    // deinit() sends a last stop frame, then releases the RMT channels and the pins.
    _escs[0]->deinit();
    _escs[1]->deinit();
    _link->idle_high();
    _entered_ms = now_ms;
    _last_activity_ms = now_ms;
    _released = false;
    set_error("");
    _state = State::TUNING;
}

void EscConfig::start_exit(uint32_t now_ms) {
    // A reset restarts each ESC out of its bootloader, then DShot takes the pins back.
    _server->reset_esc(0);
    _server->reset_esc(1);
    _link->release();
    const bool ok = _escs[0]->begin() && _escs[1]->begin();
    portENTER_CRITICAL(&_mux);
    if (_slot_state == Poll::PENDING) _slot_state = Poll::NONE;
    _error = ok ? "" : "DShot did not restart: reboot before driving";
    _state = State::EXITING;
    portEXIT_CRITICAL(&_mux);
    _exit_started_ms = now_ms;
}

void EscConfig::update(uint32_t now_ms, bool armed, bool driving, bool button_pressed) {
    portENTER_CRITICAL(&_mux);
    const int8_t requested = _requested;
    _requested = -1;
    _armed = armed;
    _driving = driving;
    portEXIT_CRITICAL(&_mux);
    const bool button_edge = button_pressed && !_prev_button;
    _prev_button = button_pressed;

    switch (_state) {
        case State::COMBAT:
            if (requested != 1) return;
            if (now_ms - _combat_since_ms < REENTRY_MS) {
                // Keep the request until the ESCs have run long enough to answer.
                portENTER_CRITICAL(&_mux);
                if (_requested == -1) _requested = 1;
                portEXIT_CRITICAL(&_mux);
                set_error("ESCs still starting up: tuning mode follows in a moment");
                return;
            }
            if (armed) {
                set_error("armed: disarm to enter tuning mode");
            } else if (driving) {
                set_error("ESP-NOW is driving: stop it to enter tuning mode");
            } else {
                enter(now_ms);
            }
            return;

        case State::TUNING: {
            bool have_frame = false;
            uint8_t frame[blheli_boot::MAX_REQUEST];
            size_t size = 0;
            portENTER_CRITICAL(&_mux);
            // Frames wait out SETTLE_MS: the ESCs need a quiet high line to enter the bootloader.
            if (_slot_state == Poll::PENDING && now_ms - _entered_ms >= SETTLE_MS) {
                memcpy(frame, _request, _request_size);
                size = _request_size;
                have_frame = true;
            }
            portEXIT_CRITICAL(&_mux);
            if (have_frame) {
                uint8_t response[blheli_boot::MAX_RESPONSE];
                const size_t n = _server->handle(frame, size, response);
                if (frame[1] == blheli_boot::DEVICE_READ && _server->first_read_failure()[0]) {
                    Serial.printf("esc tuning: read 0x%02x%02x retried after %s\n", frame[2],
                                  frame[3], _server->first_read_failure());
                }
                if (_server->resyncs() != _logged_resyncs) {
                    _logged_resyncs = _server->resyncs();
                    Serial.printf("esc tuning: redid the handshake to finish a read (%lu so far)\n",
                                  (unsigned long)_logged_resyncs);
                }
                if (_server->reboots() != _logged_reboots) {
                    _logged_reboots = _server->reboots();
                    Serial.printf(
                        "esc tuning: restarted ESC %u to reach its bootloader (%lu so far)\n",
                        frame[5], (unsigned long)_logged_reboots);
                }
                const char *failure = _server->failure();
                if (frame[1] == blheli_boot::DEVICE_INIT_FLASH) {
                    Serial.printf("esc tuning: ESC %u %s\n", frame[5], _server->init_reply());
                }
                if (n > 0 && response[5 + (response[4] == 0 ? 256 : response[4])] != 0) {
                    Serial.printf("esc tuning: cmd 0x%02x ack 0x%02x: %s\n", frame[1],
                                  response[5 + (response[4] == 0 ? 256 : response[4])], failure);
                }
                portENTER_CRITICAL(&_mux);
                strncpy(_failure, failure, sizeof(_failure) - 1);
                memcpy(_response, response, n);
                _response_size = n;
                _slot_state = Poll::DONE;
                _frames++;
                portEXIT_CRITICAL(&_mux);
                _last_activity_ms = millis();
                _released = false;
            }
            // Checked after the frame, so a write always finishes before the pins change hands.
            if (requested == 0 || button_edge) {
                start_exit(now_ms);
            } else if (!_released && millis() - _last_activity_ms >= IDLE_RELEASE_MS) {
                _server->reset_esc(0);
                _server->reset_esc(1);
                _released = true;
            }
            return;
        }

        case State::EXITING:
            if (now_ms - _exit_started_ms >= WAKE_MS) {
                _combat_since_ms = now_ms;
                _state = State::COMBAT;
            }
            return;
    }
}

}  // namespace esc_config
