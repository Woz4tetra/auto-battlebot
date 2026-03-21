#pragma once
#include <Arduino.h>

void blheli_esc_serial_init(uint8_t *pins, uint8_t pin_count);
void blheli_esc_serial_deinit();
void blheli_esc_serial_switch_pin(uint8_t index);
void blheli_esc_serial_begin();
void blheli_esc_serial_end();
uint16_t SendESC(uint8_t tx_buf[], uint16_t buf_size);
uint16_t SendESC(uint8_t tx_buf[], uint16_t buf_size, bool CRC);
uint16_t GetESC(uint8_t rx_buf[], uint16_t wait_ms);
void blheli_esc_serial_reset_pulse(uint8_t index);
uint16_t ByteCrc(uint8_t data, uint16_t crc);

extern bool Enable4Way;
extern uint8_t blheli_esc_pins[];
extern uint8_t blheli_esc_count;
