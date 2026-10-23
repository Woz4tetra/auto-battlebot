#include <SoftwareSerial.h>
#include <esc_config.h>

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

// The bootloader link on the ESC signal wires: one-wire SoftwareSerial at 19200, as in the
// standalone passthrough (firmware/BlHeli-Passthrough/ESP32/ESC_Serial.cpp), which is known to
// work with these ESCs through the 100 ohm series resistors. It runs on core 1 in loop().
class EscConfig::SerialLink : public blheli_boot::EscLink {
   public:
    SerialLink(uint8_t pin0, uint8_t pin1) : _pins{pin0, pin1} {}

    uint8_t count() const override { return 2; }

    // Restarts the serial on the pin every time, as the passthrough does: hold_low() leaves the
    // pin as a plain output.
    void select(uint8_t esc) override {
        _serial.end();
        _current = esc;
        _serial.begin(19200, SWSERIAL_8N1, _pins[esc], _pins[esc], false, 512);
        _serial.enableIntTx(false);
    }

    void send(const uint8_t *data, size_t size) override {
        if (_current < 0) return;
        _serial.enableTx(true);
        _serial.write(data, size);
        _serial.enableTx(false);
    }

    size_t receive(uint8_t *buf, size_t max, size_t expect, uint32_t first_ms) override {
        if (_current < 0) return 0;
        const uint32_t start = millis();
        while (!_serial.available()) {
            if (millis() - start >= first_ms) return 0;
            delayMicroseconds(200);
        }
        size_t n = 0;
        uint32_t last_byte = millis();
        while (n < max && (expect == 0 || n < expect)) {
            if (_serial.available()) {
                buf[n++] = _serial.read();
                last_byte = millis();
            } else if (millis() - last_byte >= 10) {
                break;
            } else {
                delayMicroseconds(100);
            }
        }
        return n;
    }

    void hold_low(uint32_t ms) override {
        if (_current < 0) return;
        const uint8_t pin = _pins[_current];
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delay(ms);
        digitalWrite(pin, HIGH);
    }

    void delay_ms(uint32_t ms) override { delay(ms); }

    // Gives both pins back for DShot.
    void release() {
        _serial.end();
        _current = -1;
        gpio_reset_pin(static_cast<gpio_num_t>(_pins[0]));
        gpio_reset_pin(static_cast<gpio_num_t>(_pins[1]));
    }

   private:
    SoftwareSerial _serial;
    uint8_t _pins[2];
    int8_t _current = -1;
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
    portEXIT_CRITICAL(&_mux);
    return s;
}

void EscConfig::enter(uint32_t now_ms) {
    // deinit() sends a last stop frame, then releases the RMT channels and the pins.
    _escs[0]->deinit();
    _escs[1]->deinit();
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
            if (_slot_state == Poll::PENDING) {
                memcpy(frame, _request, _request_size);
                size = _request_size;
                have_frame = true;
            }
            portEXIT_CRITICAL(&_mux);
            if (have_frame) {
                uint8_t response[blheli_boot::MAX_RESPONSE];
                const size_t n = _server->handle(frame, size, response);
                portENTER_CRITICAL(&_mux);
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
            if (now_ms - _exit_started_ms >= WAKE_MS) _state = State::COMBAT;
            return;
    }
}

}  // namespace esc_config
