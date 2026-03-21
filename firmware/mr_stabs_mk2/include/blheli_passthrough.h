#pragma once
#include <Arduino.h>

void passthrough_begin(uint8_t *pins, uint8_t count);
void passthrough_end();
void passthrough_process();
bool passthrough_is_active();
