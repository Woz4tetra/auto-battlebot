#pragma once
#include <Arduino.h>

typedef struct
{
    uint32_t timestamp_ms;
    bool radio_connected;
    bool armed;
    float a_percent;
    float b_percent;
    bool button_state;
    uint8_t flip_switch;
    float left_cmd;
    float right_cmd;
    float accel_x;
    float accel_y;
    float accel_z;
    bool is_upside_down;
    uint32_t loop_us;
    uint8_t wifi_clients;
} diag_data_t;

class DiagnosticsServer
{
public:
    void begin(float *channel_deadzone_ptr = nullptr);
    void update(const diag_data_t *data);

private:
    bool _recording = false;
    uint32_t _last_send_ms = 0;
    float *_channel_deadzone = nullptr;
};
