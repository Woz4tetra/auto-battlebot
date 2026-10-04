#pragma once
#include <AlfredoCRSF.h>
#include <Arduino.h>

namespace crsf_bridge {
#define RXD1 18
#define TXD1 17
#define CRSF_SERIAL Serial1

const float MIN_CYCLE = 200.0;
const float LOWER_CYCLE = 800.0;
const float MID_CYCLE = 1000.0;
const float UPPER_CYCLE = 1200.0;
const float MAX_CYCLE = 1700.0;

const float EPSILON_PERCENT = 0.1;

// Battery and attitude frames alternate, one every 50 ms, so each reaches the radio at 10 Hz.
// That matches the BNO055's 100 ms sample interval; faster would repeat the same attitude.
const uint32_t TELEMETRY_INTERVAL_MS = 50;

typedef enum three_state_switch { DOWN = 0, MIDDLE = 1, UP = 2 } three_state_switch_t;

// Values sent back to the radio as CRSF sensors. NaN fields are sent as 0.
typedef struct telemetry_data {
    float pack_volts, pack_amps;
    // BNO055 Euler angles, degrees: heading 0..360, roll, pitch.
    float heading_deg, roll_deg, pitch_deg;
} telemetry_data_t;

typedef struct radio_data {
    float a_percent, b_percent;
    bool armed, connected, button_state;
    three_state_switch_t flip_switch_state;
} radio_data_t;

class CrsfBridge {
   private:
    AlfredoCRSF *crsf;
    three_state_switch_t get_switch_state(float channel_value);
    bool get_button_state(float channel_value);
    float scale_channel_to_percent(float channel_value);
    uint32_t telemetry_timer = 0;
    bool send_attitude_next = false;
    void send_battery(const telemetry_data_t *telemetry);
    void send_attitude(const telemetry_data_t *telemetry);

   public:
    CrsfBridge();
    void begin();
    bool update(radio_data_t *radio_data);
    // Sends one telemetry frame when TELEMETRY_INTERVAL_MS has passed. Does nothing while the
    // link is down.
    void send_telemetry(const telemetry_data_t *telemetry);
};
}  // namespace crsf_bridge