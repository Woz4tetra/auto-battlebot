#include <esc.h>

using namespace esc;

Esc::Esc(gpio_num_t pin, rmt_channel_t channel, rmt_channel_t rx_channel)
    : pin(pin), channel(channel), rx_channel(rx_channel), motor(nullptr), initialized(false) {}

bool Esc::begin() {
    if (initialized) return true;
    motor = new DShot(pin, channel, rx_channel, DSHOT300);
    if (!motor->begin()) {
        delete motor;
        motor = nullptr;
        return false;
    }
    initialized = true;
    return true;
}

void Esc::deinit() {
    if (!initialized) return;
    stop();
    delete motor;
    motor = nullptr;
    initialized = false;
}

void Esc::write(float signed_percent) {
    if (!initialized) return;
    motor->send(percent_to_dshot_3d(signed_percent));
    take_erpm();
}

void Esc::stop() {
    if (!initialized) return;
    motor->send(DSHOT_STOP);
    take_erpm();
}

void Esc::send_pair(Esc &left, uint16_t left_value, Esc &right, uint16_t right_value) {
    if (!left.initialized || !right.initialized) return;
    left.motor->start(left_value);
    right.motor->start(right_value);
    left.motor->finish();
    right.motor->finish();
    left.take_erpm();
    right.take_erpm();
}

void Esc::write_pair(Esc &left, float left_percent, Esc &right, float right_percent) {
    send_pair(left, left.percent_to_dshot_3d(left_percent), right,
              right.percent_to_dshot_3d(right_percent));
}

void Esc::stop_pair(Esc &left, Esc &right) { send_pair(left, DSHOT_STOP, right, DSHOT_STOP); }

void Esc::take_erpm() {
    uint32_t erpm;
    if (motor->read_erpm(erpm)) {
        last_erpm = erpm;
        last_erpm_ms = millis();
        have_erpm = true;
    }
}

bool Esc::erpm_valid() const {
    return initialized && have_erpm && millis() - last_erpm_ms < ERPM_VALID_MS;
}

uint32_t Esc::crc_errors() const { return motor ? motor->decode_errors() : 0; }

float Esc::duty(float signed_percent) const {
    signed_percent = constrain(signed_percent, -100.0f, 100.0f);
    if (fabsf(signed_percent) < stop_threshold) return 0.0f;
    return signed_percent / 100.0f;
}

uint16_t Esc::percent_to_dshot_3d(float signed_percent) {
    signed_percent = constrain(signed_percent, -100.0f, 100.0f);
    if (fabsf(signed_percent) < stop_threshold) return DSHOT_STOP;

    if (signed_percent > 0.0f) {
        return DSHOT_3D_FWD_MIN +
               (uint16_t)(signed_percent / 100.0f * (DSHOT_3D_FWD_MAX - DSHOT_3D_FWD_MIN));
    } else {
        float magnitude = -signed_percent / 100.0f;
        return DSHOT_3D_REV_MIN + (uint16_t)(magnitude * (DSHOT_3D_REV_MAX - DSHOT_3D_REV_MIN));
    }
}
