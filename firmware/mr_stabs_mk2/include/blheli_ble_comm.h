#pragma once
#include <Arduino.h>

void blheli_ble_init();
void blheli_ble_deinit();
void blheli_ble_process();
bool blheli_ble_is_connected();
