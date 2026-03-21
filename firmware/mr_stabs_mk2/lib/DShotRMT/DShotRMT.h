//
// Name:        DShotRMT.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//
// Vendored + patched: allow sendThrottleValue(0) for motor stop in 3D mode.

#pragma once

#include <Arduino.h>

#include <driver/rmt.h>

constexpr auto DSHOT_LIB_VERSION = "0.2.4";

constexpr auto DSHOT_CLK_DIVIDER = 8;
constexpr auto DSHOT_PACKET_LENGTH = 17;
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_PAUSE = 21;
constexpr auto DSHOT_PAUSE_BIT = 16;
constexpr auto F_CPU_RMT = (80 * 1000 * 1000);
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC);

typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

static const char *const dshot_mode_name[] = {
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"};

typedef enum telemetric_request_e
{
    NO_TELEMETRIC,
    ENABLE_TELEMETRIC,
} telemetric_request_t;

typedef struct dshot_packet_s
{
    uint16_t throttle_value : 11;
    telemetric_request_t telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

typedef struct eRPM_packet_s
{
    uint16_t eRPM_data : 12;
    uint8_t checksum : 4;
} eRPM_packet_t;

typedef enum dshot_erpm_exit_mode_e
{
    DECODE_SUCCESS = 0,
    ERR_EMPTY_QUEUE,
    ERR_NO_PACKETS,
    ERR_CHECKSUM_FAIL,
    ERR_BIDIRECTION_DISABLED,

} dshot_erpm_exit_mode_t;

typedef struct dshot_config_s
{
    dshot_mode_t mode;
    String name_str;
    bool is_bidirectional;
    gpio_num_t gpio_num;
    uint8_t pin_num;
    rmt_channel_t rmt_channel;
    uint8_t mem_block_num;
    uint16_t ticks_per_bit;
    uint8_t clk_div;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
} dshot_config_t;

typedef enum dshot_cmd_e
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1,
    DSHOT_CMD_BEEP2,
    DSHOT_CMD_BEEP3,
    DSHOT_CMD_BEEP4,
    DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED,
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    DSHOT_CMD_36,
    DSHOT_CMD_37,
    DSHOT_CMD_38,
    DSHOT_CMD_39,
    DSHOT_CMD_40,
    DSHOT_CMD_41,
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY,
    DSHOT_CMD_MAX = 47
} dshot_cmd_t;

static const unsigned char GCR_encode[16] =
    {
        0x19, 0x1B, 0x12, 0x13,
        0x1D, 0x15, 0x16, 0x17,
        0x1A, 0x09, 0x0A, 0x0B,
        0x1E, 0x0D, 0x0E, 0x0F};

static const unsigned char GCR_decode[32] =
    {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 9, 10, 11,
        0xFF, 13, 14, 15,

        0xFF, 0xFF, 2, 3,
        0xFF, 5, 6, 7,
        0xFF, 0, 8, 1,
        0xFF, 4, 12, 0xFF,
};

class DShotRMT
{
public:
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel);
    DShotRMT(uint8_t pin, uint8_t channel);
    DShotRMT(uint8_t pin);

    ~DShotRMT();

    DShotRMT(DShotRMT const &);

    bool begin(dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);

    void sendThrottleValue(uint16_t throttle_value);

private:
    rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH];
    rmt_config_t dshot_tx_rmt_config;
    dshot_config_t dshot_config;

    rmt_item32_t *buildTxRmtItem(uint16_t parsed_packet);
    uint16_t calculateCRC(const dshot_packet_t &dshot_packet);
    uint16_t parseRmtPaket(const dshot_packet_t &dshot_packet);

    void sendRmtPaket(const dshot_packet_t &dshot_packet);
};
