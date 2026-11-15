#pragma once
#include <Arduino.h>

static constexpr uint8_t PIN_I2C_SDA = 2;
static constexpr uint8_t PIN_I2C_SCL = 3;
static constexpr uint8_t OLED_ADDR = 0x3C;

static constexpr uint8_t PIN_BTN_START_REC = 9;
static constexpr uint8_t PIN_BTN_STOP_REC = 6;
static constexpr uint8_t PIN_BTN_DIAGNOSTICS = 5;

static constexpr uint8_t PIN_IMU_SCK = 14;
static constexpr uint8_t PIN_IMU_MOSI = 15;
static constexpr uint8_t PIN_IMU_MISO = 8;
static constexpr uint8_t PIN_IMU_CS = 10;
static constexpr uint8_t PIN_IMU_DATA_READY = 12;

static constexpr uint8_t PIN_SD_SCK = 18;
static constexpr uint8_t PIN_SD_MOSI = 19;
static constexpr uint8_t PIN_SD_MISO = 20;
static constexpr uint8_t PIN_SD_CS = 23;

static constexpr uint8_t PIN_ENC_A = 26;
static constexpr uint8_t PIN_ENC_B = 27;

// Onboard LED (status/error blink). Not named PIN_LED: the arduino-pico variant
// already defines that as a macro.
static constexpr uint8_t PIN_STATUS_LED = 13;

static constexpr uint32_t SAMPLE_RATE_HZ = 1000;
static constexpr int64_t STRICT_SAMPLE_PERIOD_US = -(int64_t)(1000000UL / SAMPLE_RATE_HZ);
static constexpr uint32_t STREAM_PERIOD_MS = 100;
static constexpr uint32_t OLED_I2C_HZ = 400000;
static constexpr uint32_t I2C_TIMEOUT_MS = 50;

static constexpr uint8_t SPI_WRITE_MASK = 0x7F;
static constexpr uint8_t SPI_READ_BIT = 0x80;
static constexpr uint8_t REG_WHO_AM_I = 0x0F;
static constexpr uint8_t REG_CTRL3_C = 0x12;
static constexpr uint8_t REG_OUTX_L_G = 0x22;
static constexpr uint8_t REG_OUTX_L_A = 0x28;
static constexpr uint8_t ISM330DHCX_WHO_AM_I = 0x6B;
static constexpr uint8_t CTRL3_C_BDU = 0x40;
static constexpr uint8_t CTRL3_C_IF_INC = 0x04;

// IMU output data rate (the sensor's internal rate; kept above SAMPLE_RATE_HZ so
// every poll has fresh data). 1.66 kHz is a good default over SPI.
#define IMU_ODR LSM6DS_RATE_1_66K_HZ
static constexpr uint32_t IMU_ODR_HZ = 1660;

#define IMU_ACCEL_RANGE LSM6DS_ACCEL_RANGE_16_G
#define IMU_GYRO_RANGE ISM330DHCX_GYRO_RANGE_4000_DPS
static constexpr float ACCEL_G_PER_LSB = 0.000488f;
static constexpr float GYRO_DPS_PER_LSB = 0.140f;
#define ACCEL_G_PER_LSB_STR "0.000488"
#define GYRO_DPS_PER_LSB_STR "0.140"

static constexpr uint32_t SD_SCK_MHZ_VAL = 24;

static constexpr uint64_t PREALLOC_BYTES = 32ULL * 1024 * 1024;

// Capture ring buffer. Producer = ISR, consumer = main loop. Sized to absorb
// worst-case SD write stalls (~100 ms) many times over. Must be a power of two.
static constexpr uint32_t RING_SLOTS = 4096;  // 4096 * 24 B ~= 96 KB

static constexpr uint32_t WBUF_SIZE = 8192;

static constexpr uint32_t DRAIN_CAP = 1024;

static constexpr uint32_t DEBOUNCE_MS = 30;

static constexpr uint32_t DIAG_REFRESH_MS = 100;
