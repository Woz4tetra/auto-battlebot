#pragma once
#include <Arduino.h>

// MSP protocol definitions (Betaflight-compatible subset for BLHeli passthrough)
#define MSP_PROTOCOL_VERSION 0
#define API_VERSION_MAJOR 1
#define API_VERSION_MINOR 42

#define FC_VERSION_MAJOR 4
#define FC_VERSION_MINOR 1
#define FC_VERSION_PATCH_LEVEL 0

#define MSP_API_VERSION 1
#define MSP_FC_VARIANT 2
#define MSP_FC_VERSION 3
#define MSP_BOARD_INFO 4
#define MSP_BUILD_INFO 5
#define MSP_STATUS 101
#define MSP_MOTOR 104
#define MSP_MOTOR_3D_CONFIG 124
#define MSP_MOTOR_CONFIG 131
#define MSP_FEATURE_CONFIG 36
#define MSP_BOXIDS 119
#define MSP_ADVANCED_CONFIG 90
#define MSP_UID 160
#define MSP_SET_4WAY_IF 245

uint8_t MSP_Check(uint8_t MSP_buf[], uint8_t buf_size);
