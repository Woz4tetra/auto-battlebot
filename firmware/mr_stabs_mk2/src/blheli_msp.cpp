#include <Arduino.h>
#include "blheli_msp.h"
#include "blheli_esc_serial.h"

uint8_t MSP_Check(uint8_t MSP_buf[], uint8_t buf_size)
{
    uint8_t MSP_OSize = 0;
    uint8_t MSP_type = MSP_buf[4];
    uint8_t MSP_crc = MSP_buf[buf_size - 1];
    uint8_t crc;

    if (MSP_type == MSP_API_VERSION && MSP_crc == 0x01)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x03;
        MSP_buf[4] = MSP_API_VERSION;
        MSP_buf[5] = MSP_PROTOCOL_VERSION;
        MSP_buf[6] = API_VERSION_MAJOR;
        MSP_buf[7] = API_VERSION_MINOR;
        MSP_OSize = 8;
    }
    else if (MSP_type == MSP_FC_VARIANT && MSP_crc == 0x02)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x04;
        MSP_buf[4] = MSP_FC_VARIANT;
        MSP_buf[5] = 0x42; // "BTFL"
        MSP_buf[6] = 0x54;
        MSP_buf[7] = 0x46;
        MSP_buf[8] = 0x4C;
        MSP_OSize = 9;
    }
    else if (MSP_type == MSP_FC_VERSION && MSP_crc == 0x03)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x03;
        MSP_buf[4] = MSP_FC_VERSION;
        MSP_buf[5] = FC_VERSION_MAJOR;
        MSP_buf[6] = FC_VERSION_MINOR;
        MSP_buf[7] = FC_VERSION_PATCH_LEVEL;
        MSP_OSize = 8;
    }
    else if (MSP_type == MSP_BOARD_INFO && MSP_crc == 0x04)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x4F;
        MSP_buf[4] = MSP_BOARD_INFO;
        MSP_buf[5] = 'M';
        MSP_buf[6] = 'R';
        MSP_buf[7] = 'S';
        MSP_buf[8] = 'B';
        MSP_buf[9] = 0x00;
        MSP_buf[10] = 0x00;
        MSP_buf[11] = 0x00;
        MSP_buf[12] = 0x00;
        MSP_buf[13] = 0x08; // name length
        MSP_buf[14] = 'M';
        MSP_buf[15] = 'R';
        MSP_buf[16] = '-';
        MSP_buf[17] = 'S';
        MSP_buf[18] = 'T';
        MSP_buf[19] = 'A';
        MSP_buf[20] = 'B';
        MSP_buf[21] = 'S';
        for (uint8_t i = 22; i < 83; i++)
            MSP_buf[i] = 0x00;
        MSP_buf[83] = 0x00; // I2C
        MSP_OSize = 84;
    }
    else if (MSP_type == MSP_BUILD_INFO && MSP_crc == 0x05)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x1A;
        MSP_buf[4] = MSP_BUILD_INFO;
        for (uint8_t i = 5; i < 31; i++)
            MSP_buf[i] = 0x00;
        MSP_OSize = 31;
    }
    else if (MSP_type == MSP_STATUS && MSP_crc == 0x65)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x16;
        MSP_buf[4] = MSP_STATUS;
        for (uint8_t i = 5; i < 27; i++)
            MSP_buf[i] = 0x00;
        MSP_OSize = 27;
    }
    else if (MSP_type == MSP_MOTOR_3D_CONFIG && MSP_crc == 0x7C)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x06;
        MSP_buf[4] = MSP_MOTOR_3D_CONFIG;
        for (uint8_t i = 5; i < 11; i++)
            MSP_buf[i] = 0x00;
        MSP_OSize = 11;
    }
    else if (MSP_type == MSP_MOTOR_CONFIG && MSP_crc == 0x83)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x0A;
        MSP_buf[4] = MSP_MOTOR_CONFIG;
        MSP_buf[5] = 0x2E; // minthrottle 1070
        MSP_buf[6] = 0x04;
        MSP_buf[7] = 0xD0; // maxthrottle 2000
        MSP_buf[8] = 0x07;
        MSP_buf[9] = 0xE8; // mincommand 1000
        MSP_buf[10] = 0x03;
        MSP_buf[11] = blheli_esc_count; // motor count
        MSP_buf[12] = 0x00;
        MSP_buf[13] = 0x00;
        MSP_buf[14] = 0x00;
        MSP_OSize = 15;
    }
    else if (MSP_type == MSP_MOTOR && MSP_crc == 0x68)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x10;
        MSP_buf[4] = MSP_MOTOR;
        // Report idle (1000) for each motor, zero for unused slots
        for (uint8_t i = 5; i < 21; i++)
            MSP_buf[i] = 0x00;
        for (uint8_t m = 0; m < blheli_esc_count && m < 8; m++)
        {
            MSP_buf[5 + m * 2] = 0xE8; // 1000 low byte
            MSP_buf[6 + m * 2] = 0x03; // 1000 high byte
        }
        MSP_OSize = 21;
    }
    else if (MSP_type == MSP_FEATURE_CONFIG && MSP_crc == 0x24)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x04;
        MSP_buf[4] = MSP_FEATURE_CONFIG;
        for (uint8_t i = 5; i < 9; i++)
            MSP_buf[i] = 0x00;
        MSP_OSize = 9;
    }
    else if (MSP_type == MSP_BOXIDS && MSP_crc == 0x77)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x18;
        MSP_buf[4] = MSP_BOXIDS;
        for (uint8_t i = 5; i < 29; i++)
            MSP_buf[i] = 0x00;
        MSP_OSize = 29;
    }
    else if (MSP_type == MSP_SET_4WAY_IF && MSP_crc == 0xF5)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x01;
        MSP_buf[4] = MSP_SET_4WAY_IF;
        MSP_buf[5] = blheli_esc_count;
        blheli_esc_serial_begin();
        MSP_OSize = 6;
    }
    else if (MSP_type == MSP_ADVANCED_CONFIG && MSP_crc == 0x5A)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x14;
        MSP_buf[4] = MSP_ADVANCED_CONFIG;
        for (uint8_t i = 5; i < 25; i++)
            MSP_buf[i] = 0x00;
        MSP_buf[8] = 0x06; // motorPwmProtocol: DShot300
        MSP_buf[9] = 0xE0; // motorPwmRate 480
        MSP_buf[10] = 0x01;
        MSP_OSize = 25;
    }
    else if (MSP_type == MSP_UID && MSP_crc == 0xA0)
    {
        MSP_buf[2] = 0x3E;
        MSP_buf[3] = 0x0C;
        MSP_buf[4] = MSP_UID;
        for (uint8_t i = 5; i < 17; i++)
            MSP_buf[i] = 0x00;
        MSP_OSize = 17;
    }

    crc = MSP_buf[3] ^ MSP_buf[4];
    for (uint8_t i = 5; i < MSP_OSize; i++)
        crc = crc ^ MSP_buf[i];
    MSP_buf[MSP_OSize] = crc;
    buf_size = MSP_OSize + 1;

    return buf_size;
}
