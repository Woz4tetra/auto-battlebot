#include <esc.h>

using namespace esc;
using esc_map::DSHOT_STOP;

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
    motor->send(esc_map::percent_to_dshot_3d(signed_percent, stop_threshold));
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
    send_pair(left, esc_map::percent_to_dshot_3d(left_percent, left.stop_threshold), right,
              esc_map::percent_to_dshot_3d(right_percent, right.stop_threshold));
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
    return esc_map::duty(signed_percent, stop_threshold);
}
