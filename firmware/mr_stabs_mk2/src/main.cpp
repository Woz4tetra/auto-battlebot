#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <crsf_bridge.h>
#include <diagnostics_server.h>
#include <esc.h>
#include <espnow_link.h>
#include <i2c_bus.h>
#include <updown_sensor.h>
#include <vbat_sensor.h>
#include <yaw_control.h>

const char *WIFI_SSID = "MR-STABS";
const char *WIFI_PASSWORD = "havocbots";

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC_PIN ((gpio_num_t)A3)
#define RIGHT_ESC_PIN ((gpio_num_t)A2)

esc::Esc *left_esc;
esc::Esc *right_esc;

updown_sensor::UpdownSensor *accel;
vbat_sensor::VbatSensor vbat_sensor_ina228;
DiagnosticsServer diag_server;
i2c_bus::scan_t bus_scan = {};
uint32_t last_status_ms = 0;
const uint32_t STATUS_INTERVAL_MS = 1000;  // sensor panel refresh on the diagnostics page

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0, led_intensity = 20;
uint32_t last_led_ms = 0;

bool is_loading_firmware = false;
bool prev_button_state = false;
uint32_t prev_loop_us = 0;

// Steering in the flip-switch-DOWN mode: lib/yaw_control's cascaded heading and yaw-rate loop
// on the BNO055, run once per IMU sample. It replaces the heading-hold PID, which was off while
// the stick turned and so could not stop the tail-first spin. Without a usable IMU (no sample
// for HEADING_STALE_US, or the gyro failed its sign check) and while inverted, the turn stick
// passes straight through, as in MIDDLE.
yaw_control::YawController *yaw_controller;
float steer_output = 0.0f;  // differential percent: added to the left wheel, taken off the right
bool yaw_loop_active = false;
bool was_upside_down = false;
const float TURN_DEADBAND_PERCENT = 1.0f;  // passthrough steering, same as the yaw loop's
const uint32_t HEADING_STALE_US = 100000;
uint32_t last_heading_sample_us = 0;

// ESP-NOW link to the Jetson's dongle (docs/plans/espnow_link_plan.md).
espnow_link::EspNowLink espnow;

// Off until the new PCBA: this board's radio hears the dongle at -85 dBm from 30 cm, so the link
// only works at arm's length. Off means no ESP-NOW init, telemetry, or OTA; the SoftAP dashboard
// and scripts/ota_wifi are the paths in. Turning it on again also needs the ZED Box profiles back
// on the CompositeTransmitter. See "Robot radio" in the plan.
const bool ESPNOW_ENABLED = false;

// Phase 1 of the plan is shadow mode: the robot receives and reports ESP-NOW commands but keeps
// driving from the trainer channels. Phase 3 sets this true, which turns on both the command
// substitution and the slew limiter on the arbitrated output.
const bool ESPNOW_COMMAND_SUBSTITUTION = false;

// Slew-limit the arbitrated linear command so a source switch, an ESP-NOW dropout, or an
// orientation flip cannot step the wheels past the slip threshold. Same rate the Jetson applies
// (include/transmitter/config.hpp: max_motor_rpm_per_sec / max_motor_rpm = 3247 / 1500), so it is
// a no-op while the trainer path drives.
const float MAX_A_PERCENT_PER_SEC = 216.5f;
float limited_a_percent = 0.0f;

float limit_a_percent(float requested, float dt) {
    const float max_delta = MAX_A_PERCENT_PER_SEC * dt;
    limited_a_percent =
        constrain(requested, limited_a_percent - max_delta, limited_a_percent + max_delta);
    return limited_a_percent;
}

// Command-timeout failsafe
crsf_bridge::radio_data_t *prev_radio_data;
uint32_t command_timer = 0;
const uint32_t COMMAND_TIMEOUT = 5000;  // ms of identical radio frames before failsafe stop

void set_builtin_led(int value) {
    pixels.fill(pixels.Color(value, 0, 0));
    pixels.show();
}

void pulse_led() {
    for (int count = 0; count < 255; count += 5) {
        set_builtin_led(count);
        delay(1);
    }
    for (int count = 255; count > 0; count -= 5) {
        set_builtin_led(count);
        delay(1);
    }
    set_builtin_led(0);
}

void cycle_rainbow_led(int tick, int brightness) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        pixels.setPixelColor(
            i, pixels.ColorHSV(tick * 65536 / 255 + i * 65536 / NUM_PIXELS, 255, 255));
    }
    pixels.setBrightness(brightness);
    pixels.show();
}

void set_led_intensity(float percent) {
    led_intensity = (int)(2.35 * min(100.0f, max(-100.0f, percent))) + 20;
}

void stop_escs() {
    left_esc->stop();
    right_esc->stop();
    set_led_intensity(0);
    // Re-arming always ramps from standstill.
    limited_a_percent = 0.0f;
}

// Publishes one tick of diagnostics to the WiFi dashboard and, at 100 Hz, over ESP-NOW.
void publish_diag(diag_data_t &diag, bool autonomy_enabled, bool espnow_active, uint32_t now_ms) {
    diag_server.update(&diag);
    if (!ESPNOW_ENABLED) return;
    espnow_protocol::TelemetryPayload telemetry = {};
    telemetry.timestamp_ms = diag.timestamp_ms;
    telemetry.loop_us = diag.loop_us;
    telemetry.a_percent = diag.a_percent;
    telemetry.b_percent = diag.b_percent;
    telemetry.left_cmd = diag.left_cmd;
    telemetry.right_cmd = diag.right_cmd;
    telemetry.accel_x = diag.accel_x;
    telemetry.accel_y = diag.accel_y;
    telemetry.accel_z = diag.accel_z;
    telemetry.orientation_x = diag.orientation_x;
    telemetry.orientation_y = diag.orientation_y;
    telemetry.orientation_z = diag.orientation_z;
    telemetry.pid_setpoint = diag.pid_setpoint;
    telemetry.pid_output = diag.pid_output;
    telemetry.vbat = diag.vbat;
    telemetry.ibat = diag.ibat;
    telemetry.yaw_rate = diag.yaw_rate;
    telemetry.yaw_rate_cmd = diag.yaw_rate_cmd;
    updown_sensor::vector3_t *linear_accel = accel->get_linear_accel();
    telemetry.linear_accel_x = linear_accel->x;
    telemetry.linear_accel_y = linear_accel->y;
    telemetry.linear_accel_z = linear_accel->z;
    updown_sensor::vector3_t *gyro = accel->get_gyro();
    telemetry.gyro_x = gyro->x * RAD_TO_DEG;
    telemetry.gyro_y = gyro->y * RAD_TO_DEG;
    telemetry.imu_calibration = accel->get_status().calibration;
    telemetry.radio_connected = diag.radio_connected;
    telemetry.armed = diag.armed;
    telemetry.autonomy_enabled = autonomy_enabled;
    telemetry.button_state = diag.button_state;
    telemetry.flip_switch = diag.flip_switch;
    telemetry.is_upside_down = diag.is_upside_down;
    telemetry.espnow_active = espnow_active;
    espnow.send_telemetry(telemetry, now_ms);
}

// Feeds the diagnostics page's sensor and I2C panel. Does nothing with no browser open. A bus
// scan blocks for ~15 ms, so a requested scan waits until the robot is disarmed.
void publish_sensor_status(bool armed) {
    if (!diag_server.has_clients()) return;
    if (diag_server.scan_requested() && !armed) {
        i2c_bus::scan(&Wire1, &bus_scan);
        diag_server.clear_scan_request();
        last_status_ms = 0;
    }
    uint32_t now = millis();
    if (last_status_ms != 0 && now - last_status_ms < STATUS_INTERVAL_MS) return;
    last_status_ms = now;

    accel->refresh_details();
    sensor_status_t status = {};
    status.imu = accel->get_status();
    status.ina = vbat_sensor_ina228.get_status();
    status.scan = bus_scan;
    status.lines = i2c_bus::read_lines(SDA1, SCL1);
    status.sda_pin = SDA1;
    status.scl_pin = SCL1;
    status.snapshot_ms = millis();
    status.yaw_rate = accel->get_yaw_rate();
    status.heading_rate = accel->get_heading_rate();
    status.gyro_sign_suspect = accel->gyro_sign_suspect();
    diag_server.set_status(status);
}

void setup_ota() {
    ArduinoOTA
        .onStart([]() {
            is_loading_firmware = true;
            stop_escs();
        })
        .onEnd([]() {})
        .onProgress([](unsigned int progress, unsigned int total) {})
        .onError([](ota_error_t error) {});

    ArduinoOTA.begin();
}

void reset_steering(float sensed_angle_z) {
    yaw_controller->reset(sensed_angle_z);
    steer_output = 0.0f;
    yaw_loop_active = false;
}

bool compare_radio_data(const crsf_bridge::radio_data_t *data1,
                        const crsf_bridge::radio_data_t *data2) {
    return (data1->a_percent == data2->a_percent) && (data1->b_percent == data2->b_percent) &&
           (data1->armed == data2->armed) && (data1->connected == data2->connected) &&
           (data1->button_state == data2->button_state) &&
           (data1->autonomy_enabled == data2->autonomy_enabled) &&
           (data1->flip_switch_state == data2->flip_switch_state);
}

void copy_radio_data(const crsf_bridge::radio_data_t *src, crsf_bridge::radio_data_t *dest) {
    dest->a_percent = src->a_percent;
    dest->b_percent = src->b_percent;
    dest->armed = src->armed;
    dest->connected = src->connected;
    dest->button_state = src->button_state;
    dest->autonomy_enabled = src->autonomy_enabled;
    dest->flip_switch_state = src->flip_switch_state;
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float sensed_angle_z,
                       bool auto_steer_enabled, bool is_upside_down, uint32_t now_us,
                       float &left_command, float &right_command) {
    float a_percent = radio_data->a_percent;
    float b_percent = radio_data->b_percent;
    // a_percent is the negated throttle channel, so pulling the stick back reads positive, and
    // loop() negates it again when inverted. Either way, stick back drives tail-first.
    float tail_first_throttle = is_upside_down ? -a_percent : a_percent;

    uint32_t sample_us = accel->get_sample_us();
    bool fresh_heading = sample_us != 0 && sample_us != last_heading_sample_us;
    // Capped so the first sample after a gap does not integrate the whole gap.
    float heading_dt = min(sample_us - last_heading_sample_us, HEADING_STALE_US) / 1000000.0f;
    if (fresh_heading) last_heading_sample_us = sample_us;
    bool heading_stale = sample_us == 0 || now_us - sample_us > HEADING_STALE_US;

    // The differential turns the robot the same way inverted (both the wheel sides and their
    // ground direction swap), and the yaw rate is projected on the measured up direction, so
    // the loop runs upside down too. A flip restarts it: the Euler heading swings while the
    // robot goes over.
    bool yaw_usable = auto_steer_enabled && !heading_stale && !accel->gyro_sign_suspect();
    if (!yaw_usable) {
        // The stick's resting trim (about 1% on this transmitter) would otherwise creep a wheel.
        steer_output = fabs(b_percent) > TURN_DEADBAND_PERCENT ? b_percent : 0.0f;
        yaw_loop_active = false;
    } else {
        if (!yaw_loop_active || is_upside_down != was_upside_down) {
            // Engaging (switch moved to DOWN, IMU back) or just flipped: hold this heading.
            yaw_controller->reset(sensed_angle_z);
            yaw_loop_active = true;
        }
        if (fresh_heading) {
            steer_output = yaw_controller->update(b_percent, tail_first_throttle, sensed_angle_z,
                                                  accel->get_yaw_rate(), heading_dt);
        }
    }
    was_upside_down = is_upside_down;

    left_command = -1 * a_percent + steer_output;
    right_command = -1 * a_percent - steer_output;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0) {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }
}

void setup() {
    Serial.begin(115200);

#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

    left_esc = new esc::Esc(LEFT_ESC_PIN, RMT_CHANNEL_3);
    right_esc = new esc::Esc(RIGHT_ESC_PIN, RMT_CHANNEL_2);
    left_esc->stop_threshold = 1.0f;
    right_esc->stop_threshold = 1.0f;
    left_esc->begin();
    right_esc->begin();

    // ESCs need continuous DShot zero-throttle frames to initialize (~2 seconds)
    for (int i = 0; i < 400; i++) {
        left_esc->stop();
        right_esc->stop();
        delay(5);
    }

    pixels.begin();
    pixels.setBrightness(20);

    for (int count = 0; count < 2; count++) pulse_led();

    Wire1.begin();  // BNO055 IMU lives on the Wire1 I2C bus
    // Stays at the default 100 kHz. The BNO055 stretches the clock, which can fail at 400 kHz,
    // and every failed read blocks the loop for the 50 ms Wire timeout, starving the ESCs of
    // DShot frames.
    // Recorded for the diagnostics page: shows which devices answered at boot.
    i2c_bus::scan(&Wire1, &bus_scan);
    accel = new updown_sensor::UpdownSensor();
    if (!accel->begin()) {
        for (int count = 0; count < 10; count++) pulse_led();
    }
    // INA228 pack voltage and current on the same bus. If it is missing, vbat and ibat report
    // NaN and the robot runs normally.
    vbat_sensor_ina228.begin(&Wire1);
    set_builtin_led(255);

    radio_data = (crsf_bridge::radio_data_t *)malloc(sizeof(crsf_bridge::radio_data_t));
    prev_radio_data = (crsf_bridge::radio_data_t *)malloc(sizeof(crsf_bridge::radio_data_t));
    command_timer = millis();
    crsf = new crsf_bridge::CrsfBridge();
    crsf->begin();

    yaw_controller = new yaw_control::YawController(yaw_control::Config());

    // Phase 1: the SoftAP stays up as the fallback while ESP-NOW OTA proves itself, so ESP-NOW
    // runs on the AP interface and the pinned channel the AP is on.
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, espnow_protocol::CHANNEL_OVERRIDE);
    setup_ota();
    tunable_ptrs_t tunables;
    tunables.left_esc_deadzone = &left_esc->stop_threshold;
    tunables.right_esc_deadzone = &right_esc->stop_threshold;
    diag_server.begin(tunables);
    if (ESPNOW_ENABLED) espnow.begin(WIFI_IF_AP, espnow_protocol::CHANNEL_OVERRIDE, tunables);

    prev_loop_us = micros();
}

void loop() {
    uint32_t now_us = micros();
    uint32_t loop_us = now_us - prev_loop_us;
    prev_loop_us = now_us;

    ArduinoOTA.handle();
    if (is_loading_firmware) {
        return;
    }

    bool radio_ok = crsf->update(radio_data);
    uint32_t now_ms = millis();

    // An ESP-NOW firmware transfer owns the loop: motors stopped, no control, same as ArduinoOTA.
    // It only starts disarmed, and arming aborts it inside update().
    if (ESPNOW_ENABLED) {
        espnow.update(now_ms, radio_ok && radio_data->armed);
        if (espnow.ota_in_progress()) {
            stop_escs();
            return;
        }
    }
    vbat_sensor_ina228.update();
    float vbat = vbat_sensor_ina228.get_volts();
    float ibat = vbat_sensor_ina228.get_amps();

    // Read the IMU every loop with the link up, armed or not, so heading stays fresh for the PID
    // and the radio's attitude telemetry tracks the robot while it is disarmed. Rate-limited
    // inside the sensor. Skipped with the link down: get_is_upside_down(false) runs the BNO055
    // reconnect, which blocks for over a second.
    bool sensed_upside_down = radio_ok && accel->get_is_upside_down(radio_data->connected);
    updown_sensor::vector3_t *orientation = accel->get_orientation();
    crsf_bridge::telemetry_data_t telemetry = {
        .pack_volts = vbat,
        .pack_amps = ibat,
        .heading_deg = orientation->x,
        .roll_deg = orientation->y,
        .pitch_deg = orientation->z,
    };
    crsf->send_telemetry(&telemetry);
    publish_sensor_status(radio_ok && radio_data->armed);

    // Combat mode
    if (now_ms - last_led_ms >= 20) {
        last_led_ms = now_ms;
        cycle_rainbow_led(rainbow_tick, led_intensity);
        rainbow_tick = (rainbow_tick + 1) % 255;
    }

    if (!radio_ok) {
        stop_escs();
        reset_steering(accel->get_orientation()->x);

        updown_sensor::vector3_t *av = accel->get();
        updown_sensor::vector3_t *ori = accel->get_orientation();
        diag_data_t diag = {
            .timestamp_ms = millis(),
            .radio_connected = false,
            .armed = false,
            .a_percent = 0,
            .b_percent = 0,
            .button_state = false,
            .flip_switch = 0,
            .left_cmd = 0,
            .right_cmd = 0,
            .accel_x = av ? av->x : 0,
            .accel_y = av ? av->y : 0,
            .accel_z = av ? av->z : 0,
            .is_upside_down = false,
            .loop_us = loop_us,
            .wifi_clients = WiFi.softAPgetStationNum(),
            .orientation_x = ori ? ori->x : 0,
            .orientation_y = ori ? ori->y : 0,
            .orientation_z = ori ? ori->z : 0,
            .pid_setpoint = yaw_controller->setpoint(),
            .pid_output = steer_output,
            .vbat = vbat,
            .ibat = ibat,
            .yaw_rate = accel->get_yaw_rate(),
            .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
        };
        publish_diag(diag, radio_data->autonomy_enabled, false, now_ms);
        return;
    }

    // Failsafe: if the radio frame has not changed for COMMAND_TIMEOUT, treat the link as stale
    if (!compare_radio_data(radio_data, prev_radio_data)) {
        command_timer = now_ms;
        copy_radio_data(radio_data, prev_radio_data);
    } else if (now_ms - command_timer > COMMAND_TIMEOUT) {
        stop_escs();
        reset_steering(accel->get_orientation()->x);

        updown_sensor::vector3_t *av = accel->get();
        updown_sensor::vector3_t *ori = accel->get_orientation();
        diag_data_t diag = {
            .timestamp_ms = millis(),
            .radio_connected = true,
            .armed = radio_data->armed,
            .a_percent = radio_data->a_percent,
            .b_percent = radio_data->b_percent,
            .button_state = radio_data->button_state,
            .flip_switch = (uint8_t)radio_data->flip_switch_state,
            .left_cmd = 0,
            .right_cmd = 0,
            .accel_x = av ? av->x : 0,
            .accel_y = av ? av->y : 0,
            .accel_z = av ? av->z : 0,
            .is_upside_down = false,
            .loop_us = loop_us,
            .wifi_clients = WiFi.softAPgetStationNum(),
            .orientation_x = ori ? ori->x : 0,
            .orientation_y = ori ? ori->y : 0,
            .orientation_z = ori ? ori->z : 0,
            .pid_setpoint = yaw_controller->setpoint(),
            .pid_output = steer_output,
            .vbat = vbat,
            .ibat = ibat,
            .yaw_rate = accel->get_yaw_rate(),
            .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
        };
        publish_diag(diag, radio_data->autonomy_enabled, false, now_ms);
        return;
    }

    if (!radio_data->armed) {
        stop_escs();
        reset_steering(accel->get_orientation()->x);

        updown_sensor::vector3_t *av = accel->get();
        updown_sensor::vector3_t *ori = accel->get_orientation();
        diag_data_t diag = {
            .timestamp_ms = millis(),
            .radio_connected = true,
            .armed = false,
            .a_percent = radio_data->a_percent,
            .b_percent = radio_data->b_percent,
            .button_state = radio_data->button_state,
            .flip_switch = (uint8_t)radio_data->flip_switch_state,
            .left_cmd = 0,
            .right_cmd = 0,
            .accel_x = av ? av->x : 0,
            .accel_y = av ? av->y : 0,
            .accel_z = av ? av->z : 0,
            .is_upside_down = false,
            .loop_us = loop_us,
            .wifi_clients = WiFi.softAPgetStationNum(),
            .orientation_x = ori ? ori->x : 0,
            .orientation_y = ori ? ori->y : 0,
            .orientation_z = ori ? ori->z : 0,
            .pid_setpoint = yaw_controller->setpoint(),
            .pid_output = steer_output,
            .vbat = vbat,
            .ibat = ibat,
            .yaw_rate = accel->get_yaw_rate(),
            .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
        };
        publish_diag(diag, radio_data->autonomy_enabled, false, now_ms);
        return;
    }

    // ESP-NOW command substitution. Runs only after every CRSF gate has passed, so ESP-NOW can
    // never arm the robot or extend its armed window. SA (ch5) down is the pilot's "accept
    // autonomous commands" position; SA up forces the stick values on ch0/ch1.
    bool espnow_active = false;
    if (ESPNOW_ENABLED && ESPNOW_COMMAND_SUBSTITUTION && radio_data->autonomy_enabled &&
        espnow.fresh(now_ms)) {
        const espnow_protocol::CommandPayload &cmd = espnow.latest();
        // Same place the trainer path lands: the Jetson's two channel values go into ch0/ch1 at
        // full scale, and crsf_bridge negates ch0 into a_percent.
        radio_data->a_percent = -100.0f * cmd.channel_a;
        radio_data->b_percent = 100.0f * cmd.channel_b;
        espnow_active = true;
    }

    set_led_intensity((abs(radio_data->a_percent) + abs(radio_data->b_percent)) / 2.0);

    bool is_upside_down;
    bool auto_steer_enabled = false;
    switch (radio_data->flip_switch_state) {
        case crsf_bridge::UP:
            is_upside_down = true;
            break;
        case crsf_bridge::MIDDLE:
            is_upside_down = false;
            break;
        case crsf_bridge::DOWN:  // this is the default position when the transmitter powers on
            auto_steer_enabled = true;
            is_upside_down = sensed_upside_down;
            break;
        default:
            is_upside_down = false;
            break;
    }

    if (is_upside_down) radio_data->a_percent *= -1;
    // After the inversion, so a genuine flip ramps instead of reversing full scale in one tick.
    // Manual mode keeps unlimited stick authority, matching the Jetson-side limiter's gate.
    if (ESPNOW_ENABLED && ESPNOW_COMMAND_SUBSTITUTION && radio_data->autonomy_enabled) {
        radio_data->a_percent = limit_a_percent(radio_data->a_percent, loop_us / 1000000.0f);
    }

    float sensed_angle_z = orientation->x;

    float left_command, right_command;
    mix_motor_outputs(radio_data, sensed_angle_z, auto_steer_enabled, is_upside_down, micros(),
                      left_command, right_command);

    left_esc->write(left_command);
    right_esc->write(right_command);

    updown_sensor::vector3_t *av = accel->get();
    diag_data_t diag = {
        .timestamp_ms = millis(),
        .radio_connected = true,
        .armed = true,
        .a_percent = radio_data->a_percent,
        .b_percent = radio_data->b_percent,
        .button_state = radio_data->button_state,
        .flip_switch = (uint8_t)radio_data->flip_switch_state,
        .left_cmd = left_command,
        .right_cmd = right_command,
        .accel_x = av ? av->x : 0,
        .accel_y = av ? av->y : 0,
        .accel_z = av ? av->z : 0,
        .is_upside_down = is_upside_down,
        .loop_us = loop_us,
        .wifi_clients = WiFi.softAPgetStationNum(),
        .orientation_x = orientation ? orientation->x : 0,
        .orientation_y = orientation ? orientation->y : 0,
        .orientation_z = orientation ? orientation->z : 0,
        .pid_setpoint = yaw_controller->setpoint(),
        .pid_output = steer_output,
        .vbat = vbat,
        .ibat = ibat,
        .yaw_rate = accel->get_yaw_rate(),
        .yaw_rate_cmd = yaw_loop_active ? yaw_controller->rate_command() : NAN,
    };
    publish_diag(diag, radio_data->autonomy_enabled, espnow_active, now_ms);
}
