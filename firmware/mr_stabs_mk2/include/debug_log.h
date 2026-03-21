#pragma once
#include <Arduino.h>

#define DEBUG_LOG_BUF_SIZE 4096

void debug_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
const char *debug_log_get_all();
void debug_log_clear();
