#pragma once
#include <Arduino.h>

// Pin map. On arduino-pico the Arduino pin number equals the RP2040 GPIO number.
// Source of truth: docs/velocity_jig_wiring.md

// I2C1 - OLED SH1107 128x64 FeatherWing (stacked). GP2/GP3 is I2C1 -> Wire1.
static constexpr uint8_t PIN_I2C_SDA = 2;
static constexpr uint8_t PIN_I2C_SCL = 3;
static constexpr uint8_t OLED_ADDR = 0x3C;

// Buttons (active low, INPUT_PULLUP)
static constexpr uint8_t PIN_BTN_A = 9;  // start recording
static constexpr uint8_t PIN_BTN_B = 6;  // stop recording
static constexpr uint8_t PIN_BTN_C = 5;  // unused

// SPI1 - ISM330DHCX IMU
static constexpr uint8_t PIN_IMU_SCK = 14;
static constexpr uint8_t PIN_IMU_MOSI = 15;
static constexpr uint8_t PIN_IMU_MISO = 8;
static constexpr uint8_t PIN_IMU_CS = 10;
static constexpr uint8_t PIN_IMU_INT1 = 12;  // data-ready interrupt

// SPI0 - onboard microSD
static constexpr uint8_t PIN_SD_SCK = 18;
static constexpr uint8_t PIN_SD_MOSI = 19;
static constexpr uint8_t PIN_SD_MISO = 20;
static constexpr uint8_t PIN_SD_CS = 23;

// Encoder (quadrature) on A0/A1. Consecutive pins -> PIO/ISR quadrature decode.
static constexpr uint8_t PIN_ENC_A = 26;
static constexpr uint8_t PIN_ENC_B = 27;

// Onboard LED (status/error blink). Not named PIN_LED: the arduino-pico variant
// already defines that as a macro.
static constexpr uint8_t PIN_STATUS_LED = 13;

// ---- Rates and ranges ----

// IMU output data rate. INT1 data-ready drives every capture, so each logged
// row lands exactly at the ODR with no aliasing. The encoder count is sampled
// on the same interrupt. 1.66 kHz is a good default over SPI; raise if needed.
#define IMU_ODR LSM6DS_RATE_1_66K_HZ
static constexpr uint32_t IMU_ODR_HZ = 1660;

// Full-scale ranges. Battlebot: fast spin + hard hits. The standard ranges are
// the LSM6DS enums (ISM330DHCX adds only a 4000 dps gyro extension).
#define IMU_ACCEL_RANGE LSM6DS_ACCEL_RANGE_8_G
#define IMU_GYRO_RANGE LSM6DS_GYRO_RANGE_2000_DPS
// LSB scale factors for the ranges above (documented in the file header).
// String forms are used in the file header to avoid a float-printf dependency.
static constexpr float ACCEL_G_PER_LSB = 0.000244f;  // 0.244 mg/LSB at +/-8g
static constexpr float GYRO_DPS_PER_LSB = 0.070f;    // 70 mdps/LSB at +/-2000dps
#define ACCEL_G_PER_LSB_STR "0.000244"
#define GYRO_DPS_PER_LSB_STR "0.070"

// ---- SD and buffering ----

// SD SPI clock. 16 MHz is safe; many cards run 24+.
static constexpr uint32_t SD_SCK_MHZ_VAL = 16;

// Contiguous preallocation per file. Preallocating avoids FAT-growth latency
// spikes mid-run (a key part of gap-free logging). Truncated to actual size on
// close. Rows are ~60 B at ~1.66 kHz (~100 KB/s), so 32 MB covers ~5 min.
// Writes past the reservation still work but may stall; raise for longer runs.
static constexpr uint64_t PREALLOC_BYTES = 32ULL * 1024 * 1024;

// Capture ring buffer. Producer = ISR, consumer = main loop. Sized to absorb
// worst-case SD write stalls (~100 ms) many times over. Must be a power of two.
static constexpr uint32_t RING_SLOTS = 4096;  // 4096 * 24 B ~= 96 KB

// SD write staging buffer. Formatted ASCII accumulates here and flushes in
// large blocks for efficient sector writes.
static constexpr uint32_t WBUF_SIZE = 8192;

// Max records drained to SD per loop pass (keeps the STOP button responsive).
static constexpr uint32_t DRAIN_CAP = 1024;

static constexpr uint32_t DEBOUNCE_MS = 30;
