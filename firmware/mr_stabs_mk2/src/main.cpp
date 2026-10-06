#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <crsf_bridge.h>
#include <diagnostics_server.h>
#include <esc.h>
#include <i2c_bus.h>
#include <pid.h>
#include <updown_sensor.h>
#include <vbat_sensor.h>

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

// Heading hold (PID on BNO055 yaw)
pid::Pid *angle_pid;
float angle_setpoint = 0.0f;
float angle_pid_output = 0.0f;
bool was_turning = false;
float cooldown_timer = 0.0f;
bool was_auto_steer_enabled = false;
const float TURNING_COOLDOWN_TIME = 0.25f;  // coast time after a turn before heading hold engages
const float ANGULAR_SCALE = 1.0f;           // scales the manual turn command (b stick)
// Heading hold runs once per BNO055 sample, not once per loop, so its D and I terms see the real
// time between headings. With no new sample for this long, the IMU has stalled: output zero.
const uint32_t HEADING_STALE_US = 100000;
uint32_t last_heading_sample_us = 0;

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

void reset_angle_pid(float sensed_angle_z) {
    angle_pid->reset();
    angle_setpoint = sensed_angle_z;
    angle_pid_output = 0.0f;
    was_turning = false;
    cooldown_timer = 0.0f;
}

bool compare_radio_data(const crsf_bridge::radio_data_t *data1,
                        const crsf_bridge::radio_data_t *data2) {
    return (data1->a_percent == data2->a_percent) && (data1->b_percent == data2->b_percent) &&
           (data1->armed == data2->armed) && (data1->connected == data2->connected) &&
           (data1->button_state == data2->button_state) &&
           (data1->flip_switch_state == data2->flip_switch_state);
}

void copy_radio_data(const crsf_bridge::radio_data_t *src, crsf_bridge::radio_data_t *dest) {
    dest->a_percent = src->a_percent;
    dest->b_percent = src->b_percent;
    dest->armed = src->armed;
    dest->connected = src->connected;
    dest->button_state = src->button_state;
    dest->flip_switch_state = src->flip_switch_state;
}

float get_filtered_angular_z(float percent_input, float sensed_angle_z, float dt,
                             bool fresh_heading, float heading_dt, bool heading_stale) {
    float angular_v = percent_input * ANGULAR_SCALE;
    float filtered_angular_v;

    if (fabs(angular_v) > 1.0f) {
        // Actively turning: pass the turn command through and track the current heading
        filtered_angular_v = angular_v;
        angle_setpoint = sensed_angle_z;
        was_turning = true;
        cooldown_timer = TURNING_COOLDOWN_TIME;
    } else {
        if (was_turning) {
            // Just stopped turning: coast briefly before engaging heading hold
            cooldown_timer -= dt;
            if (cooldown_timer <= 0.0f) reset_angle_pid(sensed_angle_z);
        }

        if (was_turning) {
            // Still coasting during cooldown: no angular correction
            filtered_angular_v = 0.0f;
        } else if (heading_stale) {
            filtered_angular_v = 0.0f;
        } else if (fresh_heading) {
            // Hold heading
            filtered_angular_v = angle_pid->update(angle_setpoint, sensed_angle_z, heading_dt);
        } else {
            // Between samples: repeat the last correction
            filtered_angular_v = angle_pid_output;
        }
    }
    return filtered_angular_v;
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float sensed_angle_z,
                       bool auto_steer_enabled, float dt, uint32_t now_us, float &left_command,
                       float &right_command) {
    float a_percent = radio_data->a_percent;
    float b_percent = radio_data->b_percent;

    uint32_t sample_us = accel->get_sample_us();
    bool fresh_heading = sample_us != 0 && sample_us != last_heading_sample_us;
    // Capped so the first sample after arming, when last_heading_sample_us is old, does not
    // integrate the whole disarmed stretch.
    float heading_dt = min(sample_us - last_heading_sample_us, HEADING_STALE_US) / 1000000.0f;
    if (fresh_heading) last_heading_sample_us = sample_us;
    bool heading_stale = sample_us == 0 || now_us - sample_us > HEADING_STALE_US;

    if (auto_steer_enabled) {
        float filtered_angular_v = get_filtered_angular_z(b_percent, sensed_angle_z, dt,
                                                          fresh_heading, heading_dt, heading_stale);
        angle_pid_output = filtered_angular_v;
    } else {
        angle_pid_output = b_percent;
    }

    left_command = -1 * a_percent + angle_pid_output;
    right_command = -1 * a_percent - angle_pid_output;
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

    pid::PidConfig config;
    config.kp = 0.08f;
    config.ki = 0.01f;
    config.kd = 0.01f;
    config.kf = 0.0f;
    config.tolerance = 2.0f;   // hold heading within 2 degrees
    config.i_max = 20.0f;      // percent of drive command
    config.continuous = true;  // wrap yaw error across +/-180
    angle_pid = new pid::Pid(config);

    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    setup_ota();
    tunable_ptrs_t tunables;
    tunables.left_esc_deadzone = &left_esc->stop_threshold;
    tunables.right_esc_deadzone = &right_esc->stop_threshold;
    diag_server.begin(tunables);

    prev_loop_us = micros();
}

void loop() {
    uint32_t now_us = micros();
    uint32_t loop_us = now_us - prev_loop_us;
    prev_loop_us = now_us;
    float dt = loop_us / 1000000.0f;

    ArduinoOTA.handle();
    if (is_loading_firmware) {
        return;
    }

    bool radio_ok = crsf->update(radio_data);
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
    uint32_t now_ms = millis();
    if (now_ms - last_led_ms >= 20) {
        last_led_ms = now_ms;
        cycle_rainbow_led(rainbow_tick, led_intensity);
        rainbow_tick = (rainbow_tick + 1) % 255;
    }

    if (!radio_ok) {
        stop_escs();
        reset_angle_pid(accel->get_orientation()->x);

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
            .pid_setpoint = angle_setpoint,
            .pid_output = angle_pid_output,
            .vbat = vbat,
            .ibat = ibat,
        };
        diag_server.update(&diag);
        return;
    }

    // Failsafe: if the radio frame has not changed for COMMAND_TIMEOUT, treat the link as stale
    if (!compare_radio_data(radio_data, prev_radio_data)) {
        command_timer = now_ms;
        copy_radio_data(radio_data, prev_radio_data);
    } else if (now_ms - command_timer > COMMAND_TIMEOUT) {
        stop_escs();
        reset_angle_pid(accel->get_orientation()->x);

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
            .pid_setpoint = angle_setpoint,
            .pid_output = angle_pid_output,
            .vbat = vbat,
            .ibat = ibat,
        };
        diag_server.update(&diag);
        return;
    }

    if (!radio_data->armed) {
        stop_escs();
        reset_angle_pid(accel->get_orientation()->x);

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
            .pid_setpoint = angle_setpoint,
            .pid_output = angle_pid_output,
            .vbat = vbat,
            .ibat = ibat,
        };
        diag_server.update(&diag);
        return;
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

    float sensed_angle_z = orientation->x;
    if (auto_steer_enabled != was_auto_steer_enabled && auto_steer_enabled) {
        reset_angle_pid(sensed_angle_z);
    }
    was_auto_steer_enabled = auto_steer_enabled;

    float left_command, right_command;
    mix_motor_outputs(radio_data, sensed_angle_z, auto_steer_enabled, dt, micros(), left_command,
                      right_command);

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
        .pid_setpoint = angle_setpoint,
        .pid_output = angle_pid_output,
        .vbat = vbat,
        .ibat = ibat,
    };
    diag_server.update(&diag);
}
