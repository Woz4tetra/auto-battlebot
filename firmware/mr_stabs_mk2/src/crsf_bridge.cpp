#include <crsf_bridge.h>

using namespace crsf_bridge;

CrsfBridge::CrsfBridge() { crsf = new AlfredoCRSF(); }

void CrsfBridge::begin() {
    CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1, RXD1, TXD1);
    crsf->begin(CRSF_SERIAL);
}

bool CrsfBridge::update(radio_data_t *radio_data) {
    crsf->update();
    three_state_switch_t flip_switch_state;
    if (crsf->isLinkUp()) {
        const crsf_channels_t *channels = crsf->getChannelsPacked();
        radio_data->a_percent = -1 * scale_channel_to_percent(channels->ch0);
        radio_data->b_percent = scale_channel_to_percent(channels->ch1);
        radio_data->armed = channels->ch4 > MID_CYCLE;
        radio_data->flip_switch_state = get_switch_state(channels->ch7);
        radio_data->button_state = get_button_state(channels->ch9);
        radio_data->connected = true;
        return true;
    } else {
        radio_data->a_percent = 0.0;
        radio_data->b_percent = 0.0;
        radio_data->armed = false;
        radio_data->flip_switch_state = MIDDLE;
        radio_data->button_state = false;
        radio_data->connected = false;
        return false;
    }
}

float CrsfBridge::scale_channel_to_percent(float channel_value) {
    float percent;
    if (channel_value < MID_CYCLE)
        percent = -100.0 / (MID_CYCLE - MIN_CYCLE) * (MID_CYCLE - channel_value);
    else
        percent = 100.0 / (MAX_CYCLE - MID_CYCLE) * (channel_value - MID_CYCLE);
    if (abs(percent) < EPSILON_PERCENT) percent = 0.0;
    return min(100.0f, max(-100.0f, percent));
}

three_state_switch_t CrsfBridge::get_switch_state(float channel_value) {
    if (channel_value < LOWER_CYCLE)
        return DOWN;
    else if (channel_value < UPPER_CYCLE)
        return MIDDLE;
    else
        return UP;
}

bool CrsfBridge::get_button_state(float channel_value) { return channel_value > MID_CYCLE; }

// CRSF sensor frames are big-endian. NaN and out-of-range values are clamped to the field.
static uint16_t to_be_u16(float value) {
    if (!(value > 0.0f)) return 0;
    return htobe16((uint16_t)min(value, 65535.0f));
}

static int16_t to_be_i16(float value) {
    if (isnan(value)) return 0;
    return htobe16((int16_t)min(32767.0f, max(-32768.0f, value)));
}

static float wrap_degrees(float degrees) {
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

void CrsfBridge::send_telemetry(const telemetry_data_t *telemetry) {
    if (!crsf->isLinkUp()) return;
    uint32_t now = millis();
    if (now - telemetry_timer < TELEMETRY_INTERVAL_MS) return;
    telemetry_timer = now;
    if (send_attitude_next)
        send_attitude(telemetry);
    else
        send_battery(telemetry);
    send_attitude_next = !send_attitude_next;
}

void CrsfBridge::send_battery(const telemetry_data_t *telemetry) {
    // Capacity used and percent remaining are not tracked yet, so they go out as 0.
    crsf_sensor_battery_t battery = {0};
    battery.voltage = to_be_u16(telemetry->pack_volts * 10.0f);
    battery.current = to_be_u16(telemetry->pack_amps * 10.0f);
    crsf->queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery, sizeof(battery));
}

void CrsfBridge::send_attitude(const telemetry_data_t *telemetry) {
    // Radians x 10000 in an int16, so every angle must be within +-3.27 rad. Heading is
    // wrapped from 0..360 to +-180 degrees first.
    const float scale = 10000.0f * DEG_TO_RAD;
    crsf_sensor_attitude_t attitude = {0};
    attitude.pitch = to_be_i16(telemetry->pitch_deg * scale);
    attitude.roll = to_be_i16(telemetry->roll_deg * scale);
    attitude.yaw = to_be_i16(wrap_degrees(telemetry->heading_deg) * scale);
    crsf->queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_ATTITUDE, &attitude, sizeof(attitude));
}
