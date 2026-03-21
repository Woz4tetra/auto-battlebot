#pragma once
#include <Arduino.h>

#define SERIAL_4WAY_PROTOCOL_VER 108
#define SERIAL_4WAY_VER_MAIN 20
#define SERIAL_4WAY_VER_SUB_1 (uint8_t)0
#define SERIAL_4WAY_VER_SUB_2 (uint8_t)05
#define SERIAL_4WAY_VERSION (uint16_t)((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)
#define SERIAL_4WAY_VERSION_HI (uint8_t)(SERIAL_4WAY_VERSION / 100)
#define SERIAL_4WAY_VERSION_LO (uint8_t)(SERIAL_4WAY_VERSION % 100)

#define imARM_BLB 4

#define cmd_Remote_Escape 0x2E
#define cmd_Local_Escape 0x2F
#define cmd_InterfaceTestAlive 0x30
#define cmd_ProtocolGetVersion 0x31
#define cmd_InterfaceGetName 0x32
#define cmd_InterfaceGetVersion 0x33
#define cmd_InterfaceExit 0x34
#define cmd_DeviceReset 0x35
#define cmd_DeviceInitFlash 0x37
#define cmd_DeviceEraseAll 0x38
#define cmd_DevicePageErase 0x39
#define cmd_DeviceRead 0x3A
#define cmd_DeviceWrite 0x3B
#define cmd_DeviceC2CK_LOW 0x3C
#define cmd_DeviceReadEEprom 0x3D
#define cmd_DeviceWriteEEprom 0x3E
#define cmd_InterfaceSetMode 0x3F
#define cmd_DeviceVerify 0x40

#define ACK_OK 0x00
#define ACK_I_INVALID_CMD 0x02
#define ACK_I_INVALID_CRC 0x03
#define ACK_I_VERIFY_ERROR 0x04
#define ACK_I_INVALID_CHANNEL 0x08
#define ACK_I_INVALID_PARAM 0x09
#define ACK_D_GENERAL_ERROR 0x0F

#define RestartBootloader 0
#define ExitBootloader 1

#define CMD_RUN 0x00
#define CMD_PROG_FLASH 0x01
#define CMD_ERASE_FLASH 0x02
#define CMD_READ_FLASH_SIL 0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM 0x04
#define CMD_PROG_EEPROM 0x05
#define CMD_READ_SRAM 0x06
#define CMD_READ_FLASH_ATM 0x07
#define CMD_KEEP_ALIVE 0xFD
#define CMD_SET_ADDRESS 0xFF
#define CMD_SET_BUFFER 0xFE

#define brSUCCESS 0x30
#define brERRORVERIFY 0xC0
#define brERRORCOMMAND 0xC1
#define brERRORCRC 0xC2
#define brNONE 0xFF

uint16_t Check_4Way(uint8_t buf[]);
uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data);
