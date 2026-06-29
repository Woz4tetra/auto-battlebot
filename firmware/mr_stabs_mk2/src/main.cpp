#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <crsf_bridge.h>
#include <diagnostics_server.h>
#include <esc.h>
#include <pid.h>
#include <updown_sensor.h>

const char *WIFI_SSID = "MR-STABS";
const char *WIFI_PASSWORD = "havocbots";

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC_PIN ((gpio_num_t)A3)
#define RIGHT_ESC_PIN ((gpio_num_t)A2)

esc::Esc *left_esc;
esc::Esc *right_esc;

updown_sensor::UpdownSensor *accel;
DiagnosticsServer diag_server;

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
const float TURNING_COOLDOWN_TIME = 0.25f;  // coast time after a turn before heading hold engages
const float ANGULAR_SCALE = 1.0f;           // scales the manual turn command (b stick)

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
           (data1->c_percent == data2->c_percent) &&
           (data1->lifter_command == data2->lifter_command) && (data1->armed == data2->armed) &&
           (data1->connected == data2->connected) && (data1->button_state == data2->button_state) &&
           (data1->flip_switch_state == data2->flip_switch_state);
}

void copy_radio_data(const crsf_bridge::radio_data_t *src, crsf_bridge::radio_data_t *dest) {
    dest->a_percent = src->a_percent;
    dest->b_percent = src->b_percent;
    dest->c_percent = src->c_percent;
    dest->lifter_command = src->lifter_command;
    dest->armed = src->armed;
    dest->connected = src->connected;
    dest->button_state = src->button_state;
    dest->flip_switch_state = src->flip_switch_state;
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float sensed_angle_z, float dt,
                       float &left_command, float &right_command) {
    float angular_v = radio_data->b_percent * ANGULAR_SCALE;
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
        } else {
            // Hold heading
            filtered_angular_v = angle_pid->update(angle_setpoint, sensed_angle_z, dt);
        }
    }

    angle_pid_output = filtered_angular_v;

    left_command = -1 * radio_data->a_percent + filtered_angular_v;
    right_command = -1 * radio_data->a_percent - filtered_angular_v;
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
    accel = new updown_sensor::UpdownSensor();
    if (!accel->begin()) {
        for (int count = 0; count < 10; count++) pulse_led();
    }
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
    config.tolerance = 2.0f;  // hold heading within 2 degrees
    config.i_max = 1000.0f;
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

    if (is_loading_firmware) {
        ArduinoOTA.handle();
        return;
    }

    bool radio_ok = crsf->update(radio_data);

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
        };
        diag_server.update(&diag);
        return;
    }

    ArduinoOTA.handle();

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
        };
        diag_server.update(&diag);
        return;
    }

    set_led_intensity((abs(radio_data->a_percent) + abs(radio_data->b_percent)) / 2.0);

    // Read the IMU every armed loop so heading stays fresh for the PID, regardless of flip mode
    bool sensed_upside_down = accel->get_is_upside_down(radio_data->connected);
    bool is_upside_down;
    switch (radio_data->flip_switch_state) {
        case crsf_bridge::UP:
            is_upside_down = true;
            break;
        case crsf_bridge::MIDDLE:
            is_upside_down = false;
            break;
        case crsf_bridge::DOWN:
            is_upside_down = sensed_upside_down;
            break;
        default:
            is_upside_down = false;
            break;
    }

    if (is_upside_down) radio_data->a_percent *= -1;

    updown_sensor::vector3_t *orientation = accel->get_orientation();
    float sensed_angle_z = orientation->x;

    float left_command, right_command;
    mix_motor_outputs(radio_data, sensed_angle_z, dt, left_command, right_command);

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
    };
    diag_server.update(&diag);
}
