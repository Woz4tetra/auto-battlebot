#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <crsf_bridge.h>
#include <esc.h>
#include <updown_sensor.h>
#include <diagnostics_server.h>

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

void set_builtin_led(int value)
{
    pixels.fill(pixels.Color(value, 0, 0));
    pixels.show();
}

void pulse_led()
{
    for (int count = 0; count < 255; count += 5)
    {
        set_builtin_led(count);
        delay(1);
    }
    for (int count = 255; count > 0; count -= 5)
    {
        set_builtin_led(count);
        delay(1);
    }
    set_builtin_led(0);
}

void cycle_rainbow_led(int tick, int brightness)
{
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        pixels.setPixelColor(i, pixels.ColorHSV(tick * 65536 / 255 + i * 65536 / NUM_PIXELS, 255, 255));
    }
    pixels.setBrightness(brightness);
    pixels.show();
}

void set_led_intensity(float percent)
{
    led_intensity = (int)(2.35 * min(100.0f, max(-100.0f, percent))) + 20;
}

void stop_escs()
{
    left_esc->stop();
    right_esc->stop();
    set_led_intensity(0);
}

void setup_ota()
{
    ArduinoOTA
        .onStart([]()
                 {
    is_loading_firmware = true;
    stop_escs(); })
        .onEnd([]() {})
        .onProgress([](unsigned int progress, unsigned int total) {})
        .onError([](ota_error_t error) {});

    ArduinoOTA.begin();
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float &left_command, float &right_command)
{
    left_command = -1 * radio_data->a_percent + radio_data->b_percent;
    right_command = -1 * radio_data->a_percent - radio_data->b_percent;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0)
    {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }
}

void setup()
{
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
    for (int i = 0; i < 400; i++)
    {
        left_esc->stop();
        right_esc->stop();
        delay(5);
    }

    pixels.begin();
    pixels.setBrightness(20);

    for (int count = 0; count < 2; count++)
        pulse_led();

    accel = new updown_sensor::UpdownSensor();
    if (!accel->begin())
    {
        for (int count = 0; count < 10; count++)
            pulse_led();
    }
    set_builtin_led(255);

    radio_data = (crsf_bridge::radio_data_t *)malloc(sizeof(crsf_bridge::radio_data_t));
    crsf = new crsf_bridge::CrsfBridge();
    crsf->begin();

    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    setup_ota();
    tunable_ptrs_t tunables;
    tunables.left_esc_deadzone = &left_esc->stop_threshold;
    tunables.right_esc_deadzone = &right_esc->stop_threshold;
    diag_server.begin(tunables);

    prev_loop_us = micros();
}

void loop()
{
    uint32_t now_us = micros();
    uint32_t loop_us = now_us - prev_loop_us;
    prev_loop_us = now_us;

    if (is_loading_firmware)
    {
        ArduinoOTA.handle();
        return;
    }

    bool radio_ok = crsf->update(radio_data);

    // Combat mode
    uint32_t now_ms = millis();
    if (now_ms - last_led_ms >= 20)
    {
        last_led_ms = now_ms;
        cycle_rainbow_led(rainbow_tick, led_intensity);
        rainbow_tick = (rainbow_tick + 1) % 255;
    }

    if (!radio_ok)
    {
        stop_escs();

        updown_sensor::vector3_t *av = accel->get();
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
        };
        diag_server.update(&diag);
        return;
    }

    ArduinoOTA.handle();

    if (!radio_data->armed)
    {
        stop_escs();

        updown_sensor::vector3_t *av = accel->get();
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
        };
        diag_server.update(&diag);
        return;
    }

    set_led_intensity((abs(radio_data->a_percent) + abs(radio_data->b_percent)) / 2.0);

    bool is_upside_down;
    switch (radio_data->flip_switch_state)
    {
    case crsf_bridge::UP:
        is_upside_down = true;
        break;
    case crsf_bridge::MIDDLE:
        is_upside_down = false;
        break;
    case crsf_bridge::DOWN:
        is_upside_down = accel->get_is_upside_down(radio_data->connected);
        break;
    default:
        is_upside_down = false;
        break;
    }

    if (is_upside_down)
        radio_data->a_percent *= -1;

    float left_command, right_command;
    mix_motor_outputs(radio_data, left_command, right_command);

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
    };
    diag_server.update(&diag);
}
