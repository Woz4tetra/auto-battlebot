#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <crsf_bridge.h>
#include <esc.h>
#include <updown_sensor.h>
#include <blheli_passthrough.h>

#define MAIN_SERIAL Serial

const char *WIFI_SSID = "MR-STABS";
const char *WIFI_PASSWORD = "havocbots";

crsf_bridge::CrsfBridge *crsf;
crsf_bridge::radio_data_t *radio_data;

#define LEFT_ESC_PIN ((gpio_num_t)A2)
#define RIGHT_ESC_PIN ((gpio_num_t)A3)

esc::Esc *left_esc;
esc::Esc *right_esc;

updown_sensor::UpdownSensor *accel;

const int NUM_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int rainbow_tick = 0, led_intensity = 20;

bool is_loading_firmware = false;
bool tuning_active = false;
bool prev_button_state = false;

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
    stop_escs();
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    MAIN_SERIAL.println("Start updating " + type); })
        .onEnd([]()
               { MAIN_SERIAL.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { MAIN_SERIAL.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
    MAIN_SERIAL.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) MAIN_SERIAL.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) MAIN_SERIAL.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) MAIN_SERIAL.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) MAIN_SERIAL.println("Receive Failed");
    else if (error == OTA_END_ERROR) MAIN_SERIAL.println("End Failed"); });

    ArduinoOTA.begin();
}

void enter_tuning_mode()
{
    MAIN_SERIAL.println("Entering tuning mode");
    left_esc->deinit();
    right_esc->deinit();
    uint8_t pins[] = {(uint8_t)LEFT_ESC_PIN, (uint8_t)RIGHT_ESC_PIN};
    passthrough_begin(pins, 2);
    tuning_active = true;
}

void exit_tuning_mode()
{
    MAIN_SERIAL.println("Exiting tuning mode");
    passthrough_end();
    left_esc->begin();
    right_esc->begin();
    tuning_active = false;
}

void mix_motor_outputs(crsf_bridge::radio_data_t *radio_data, float &left_command, float &right_command)
{
    left_command = radio_data->a_percent + radio_data->b_percent;
    right_command = radio_data->a_percent - radio_data->b_percent;
    float max_command = max(abs(left_command), abs(right_command));
    if (max_command > 100.0)
    {
        left_command = left_command / max_command * 100.0;
        right_command = right_command / max_command * 100.0;
    }
}

void setup()
{
    MAIN_SERIAL.begin(115200);
    MAIN_SERIAL.println("Starting setup");

#if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    MAIN_SERIAL.println("Set neopixel power");
#endif

    left_esc = new esc::Esc(LEFT_ESC_PIN, RMT_CHANNEL_0);
    right_esc = new esc::Esc(RIGHT_ESC_PIN, RMT_CHANNEL_1);
    left_esc->begin();
    right_esc->begin();
    delay(500);

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

    MAIN_SERIAL.println("Setup complete");
}

void loop()
{
    delay(10);
    cycle_rainbow_led(rainbow_tick, led_intensity);
    rainbow_tick = (rainbow_tick + 5) % 255;

    if (is_loading_firmware)
    {
        ArduinoOTA.handle();
        return;
    }

    if (!crsf->update(radio_data))
    {
        MAIN_SERIAL.println("Disconnected from radio");
        if (!tuning_active)
            stop_escs();
        return;
    }

    // Rising-edge detection on button_state for toggle
    bool button_now = radio_data->button_state;
    if (button_now && !prev_button_state)
    {
        if (tuning_active)
            exit_tuning_mode();
        else
            enter_tuning_mode();
    }
    prev_button_state = button_now;

    if (tuning_active)
    {
        passthrough_process();
        ArduinoOTA.handle();
        return;
    }

    // Combat mode
    ArduinoOTA.handle();

    if (!radio_data->armed)
    {
        MAIN_SERIAL.println("Disarmed.");
        stop_escs();
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
}
