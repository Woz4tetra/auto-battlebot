#include <Arduino.h>

uint8_t ble_rx[300] = {0};          
uint8_t serial_rx[300] = {0};

bool Enable4Way = false;
uint8_t esc_pins[2] = {A2, A3};
uint8_t esc_pin_count = 2;
uint8_t esc_pin_index = 0;
