#include <Arduino.h>
#include "blheli_4way.h"
#include "blheli_esc_serial.h"
#include "debug_log.h"

uint16_t Check_4Way(uint8_t buf[])
{
    uint8_t cmd = buf[1];
    uint8_t addr_high = buf[2];
    uint8_t addr_low = buf[3];
    uint8_t I_Param_Len = buf[4];
    uint8_t param = buf[5];
    uint8_t ParamBuf[256] = {0};
    uint16_t crc = 0;
    uint16_t buf_size;
    uint8_t ack_out = ACK_OK;
    uint16_t O_Param_Len = 0;

    for (uint8_t i = 0; i < 5; i++)
        crc = _crc_xmodem_update(crc, buf[i]);

    uint8_t InBuff = I_Param_Len;
    uint16_t i = 0;
    do
    {
        crc = _crc_xmodem_update(crc, buf[i + 5]);
        ParamBuf[i] = buf[i + 5];
        i++;
        InBuff--;
    } while (InBuff != 0);
    uint16_t crc_in = ((buf[i + 5] << 8) | buf[i + 6]);

    if (crc_in != crc)
    {
        debug_log("4WAY CRC FAIL cmd=0x%02X exp=0x%04X got=0x%04X", cmd, crc, crc_in);
        buf[0] = cmd_Remote_Escape;
        buf[1] = cmd;
        buf[2] = addr_high;
        buf[3] = addr_low;
        O_Param_Len = 0x01;
        buf[4] = 0x01;
        buf[5] = 0x00;
        ack_out = ACK_I_INVALID_CRC;
        buf[6] = ACK_I_INVALID_CRC;
        crc = 0;
        for (uint8_t i = 0; i < 7; i++)
            crc = _crc_xmodem_update(crc, buf[i]);
        buf[7] = (crc >> 8) & 0xff;
        buf[8] = crc & 0xff;
        buf_size = 9;
        if (cmd < 0x50)
            return buf_size;
    }

    crc = 0;
    ack_out = ACK_OK;
    buf[5] = 0;

    if (cmd == cmd_DeviceInitFlash)
    {
        debug_log("DeviceInitFlash ESC#%u", param);
        O_Param_Len = 0x04;
        if (param < blheli_esc_count)
        {
            blheli_esc_serial_switch_pin(param);
            uint8_t BootInit[] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
            uint8_t Init_Size = 17;
            uint8_t RX_Buf[250] = {0};
            uint16_t RX_Size = 0;
            debug_log("  sending BootInit %u bytes", Init_Size);
            SendESC(BootInit, Init_Size, false);
            delay(80);
            RX_Size = GetESC(RX_Buf, 300);
            if (RX_Size > 0 && RX_Size < 7)
            {
                debug_log("  partial %u bytes, reading more", RX_Size);
                delay(50);
                RX_Size += GetESC(RX_Buf + RX_Size, 200);
            }
            debug_log("  GetESC %u bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                      RX_Size,
                      RX_Size > 0 ? RX_Buf[0] : 0, RX_Size > 1 ? RX_Buf[1] : 0,
                      RX_Size > 2 ? RX_Buf[2] : 0, RX_Size > 3 ? RX_Buf[3] : 0,
                      RX_Size > 4 ? RX_Buf[4] : 0, RX_Size > 5 ? RX_Buf[5] : 0,
                      RX_Size > 6 ? RX_Buf[6] : 0, RX_Size > 7 ? RX_Buf[7] : 0,
                      RX_Size > 8 ? RX_Buf[8] : 0);
            if (RX_Size >= 7 && RX_Buf[RX_Size - 1] == brSUCCESS)
            {
                buf[5] = RX_Buf[RX_Size - 4];
                buf[6] = RX_Buf[RX_Size - 5];
                buf[7] = 0;
                buf[8] = imARM_BLB;
                buf[9] = ACK_OK;
                debug_log("  InitFlash OK sig=%02X%02X mode=%u", buf[6], buf[5], buf[8]);
            }
            else
            {
                debug_log("  InitFlash FAIL");
                buf[5] = 0x06;
                buf[6] = 0x33;
                buf[7] = 0x67;
                buf[8] = imARM_BLB;
                ack_out = ACK_D_GENERAL_ERROR;
                buf[9] = ACK_D_GENERAL_ERROR;
            }
        }
        else
        {
            debug_log("  invalid channel %u (count=%u)", param, blheli_esc_count);
            ack_out = ACK_I_INVALID_CHANNEL;
            buf[9] = ACK_I_INVALID_CHANNEL;
        }
    }

    else if (cmd == cmd_DeviceReset)
    {
        debug_log("DeviceReset ESC#%u", param);
        O_Param_Len = 0x01;
        if (param < blheli_esc_count)
        {
            buf[6] = ACK_OK;
            if (Enable4Way)
            {
                debug_log("  sending RestartBootloader");
                uint8_t ESC_data[2] = {RestartBootloader, 0};
                SendESC(ESC_data, 2);
            }
            debug_log("  end serial, pulse pin %u LOW 300ms", blheli_esc_pins[param]);
            blheli_esc_serial_end();
            uint8_t pin = blheli_esc_pins[param];
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
            delay(300);
            digitalWrite(pin, HIGH);
            delay(10);
            debug_log("  reopen serial on ESC#%u", param);
            blheli_esc_serial_switch_pin(param);
            delay(100);
            debug_log("  DeviceReset done");
        }
        else
        {
            debug_log("  invalid channel %u", param);
            buf[5] = 0x00;
            ack_out = ACK_I_INVALID_CHANNEL;
            buf[6] = ACK_I_INVALID_CHANNEL;
        }
    }

    else if (cmd == cmd_InterfaceTestAlive)
    {
        debug_log("InterfaceTestAlive");
        O_Param_Len = 0x01;
        uint8_t ESC_data[2] = {CMD_KEEP_ALIVE, 0};
        uint8_t RX_Buf[250] = {0};
        SendESC(ESC_data, 2);
        delay(5);
        GetESC(RX_Buf, 200);
    }

    else if (cmd == cmd_DeviceRead)
    {
        debug_log("DeviceRead addr=0x%02X%02X len=%u", addr_high, addr_low, param);
        uint8_t ESC_data[4] = {CMD_SET_ADDRESS, 0x00, addr_high, addr_low};
        uint16_t RX_Size = 0;
        uint8_t RX_Buf[300] = {0};
        uint16_t esc_rx_crc = 0;
        SendESC(ESC_data, 4);
        delay(5);
        RX_Size = GetESC(RX_Buf, 200);
        if (RX_Buf[0] == brSUCCESS)
        {
            ESC_data[0] = CMD_READ_FLASH_SIL;
            ESC_data[1] = param;
            SendESC(ESC_data, 2);
            if (param == 0)
            {
                O_Param_Len = 256;
                delay(256);
            }
            else
            {
                delay(param);
                O_Param_Len = param;
            }
            RX_Size = GetESC(RX_Buf, 500);
            if (RX_Size != 0)
            {
                if (RX_Buf[(RX_Size - 1)] != brSUCCESS)
                    ack_out = ACK_D_GENERAL_ERROR;

                RX_Size = RX_Size - 3;
                O_Param_Len = RX_Size;
                for (uint16_t i = 5; i < (RX_Size + 5); i++)
                {
                    buf[i] = RX_Buf[i - 5];
                    esc_rx_crc = ByteCrc(buf[i], esc_rx_crc);
                }
                esc_rx_crc = ByteCrc(RX_Buf[RX_Size], esc_rx_crc);
                esc_rx_crc = ByteCrc(RX_Buf[RX_Size + 1], esc_rx_crc);
                if (esc_rx_crc != 0)
                {
                    ack_out = ACK_D_GENERAL_ERROR;
                    O_Param_Len = 0x01;
                }
            }
            else
            {
                ack_out = ACK_D_GENERAL_ERROR;
                O_Param_Len = 0x01;
            }
        }
        else
        {
            O_Param_Len = 0x01;
            ack_out = ACK_D_GENERAL_ERROR;
        }
    }

    else if (cmd == cmd_InterfaceExit)
    {
        debug_log("InterfaceExit");
        blheli_esc_serial_end();
        O_Param_Len = 0x01;
    }

    else if (cmd == cmd_ProtocolGetVersion)
    {
        O_Param_Len = 0x01;
        buf[5] = SERIAL_4WAY_PROTOCOL_VER;
    }

    else if (cmd == cmd_InterfaceGetName)
    {
        O_Param_Len = 0x09;
        buf[5] = 'm';
        buf[6] = '4';
        buf[7] = 'w';
        buf[8] = 'F';
        buf[9] = 'C';
        buf[10] = 'I';
        buf[11] = 'n';
        buf[12] = 't';
        buf[13] = 'f';
    }

    else if (cmd == cmd_InterfaceGetVersion)
    {
        O_Param_Len = 0x02;
        buf[5] = SERIAL_4WAY_VERSION_HI;
        buf[6] = SERIAL_4WAY_VERSION_LO;
    }

    else if (cmd == cmd_InterfaceSetMode)
    {
        debug_log("InterfaceSetMode param=0x%02X", param);
        O_Param_Len = 0x01;
        if (param != imARM_BLB)
        {
            buf[6] = ACK_I_INVALID_PARAM;
            ack_out = ACK_I_INVALID_PARAM;
        }
    }

    else if (cmd == cmd_DeviceVerify)
    {
        O_Param_Len = 0x01;
    }

    else if (cmd == cmd_DevicePageErase)
    {
        debug_log("DevicePageErase page=%u", param);
        O_Param_Len = 0x01;
        addr_high = (param << 2);
        addr_low = 0;
        uint8_t ESC_data[4] = {CMD_SET_ADDRESS, 0, addr_high, addr_low};
        uint8_t RX_Buf[250] = {0};
        uint16_t RX_Size = 0;
        SendESC(ESC_data, 4);
        delay(5);
        RX_Size = GetESC(RX_Buf, 200);
        if (RX_Buf[0] != brSUCCESS)
        {
            ack_out = ACK_D_GENERAL_ERROR;
            buf[6] = ACK_D_GENERAL_ERROR;
        }
        ESC_data[0] = CMD_ERASE_FLASH;
        ESC_data[1] = 0x01;
        SendESC(ESC_data, 2);
        delay(50);
        RX_Size = GetESC(RX_Buf, 100);
        if (RX_Buf[0] != brSUCCESS)
        {
            ack_out = ACK_D_GENERAL_ERROR;
            buf[6] = ACK_D_GENERAL_ERROR;
        }
    }

    else if (cmd == cmd_DeviceWrite)
    {
        debug_log("DeviceWrite addr=0x%02X%02X len=%u", addr_high, addr_low, I_Param_Len);
        O_Param_Len = 0x01;
        uint8_t ESC_data[4] = {CMD_SET_ADDRESS, 0, addr_high, addr_low};
        uint8_t RX_Buf[250] = {0};
        uint16_t RX_Size = 0;
        SendESC(ESC_data, 4);
        delay(50);
        RX_Size = GetESC(RX_Buf, 100);
        if (RX_Buf[0] != brSUCCESS)
            ack_out = ACK_D_GENERAL_ERROR;

        ESC_data[0] = CMD_SET_BUFFER;
        ESC_data[1] = 0x00;
        ESC_data[2] = 0x00;
        ESC_data[3] = I_Param_Len;
        if (I_Param_Len == 0)
            ESC_data[2] = 0x01;
        SendESC(ESC_data, 4);
        delay(5);

        SendESC(ParamBuf, I_Param_Len);
        delay(5);
        RX_Size = GetESC(RX_Buf, 200);
        if (RX_Buf[0] != brSUCCESS)
            ack_out = ACK_D_GENERAL_ERROR;

        ESC_data[0] = CMD_PROG_FLASH;
        ESC_data[1] = 0x01;
        SendESC(ESC_data, 2);
        delay(30);
        RX_Size = GetESC(RX_Buf, 100);
        if (RX_Buf[0] != brSUCCESS)
            ack_out = ACK_D_GENERAL_ERROR;
    }

    else
    {
        debug_log("4WAY unknown cmd 0x%02X", cmd);
        buf_size = 0;
    }

    debug_log("4WAY reply cmd=0x%02X ack=0x%02X oLen=%u", cmd, ack_out, O_Param_Len);
    crc = 0;
    buf[0] = cmd_Remote_Escape;
    buf[1] = cmd;
    buf[2] = addr_high;
    buf[3] = addr_low;
    buf[4] = O_Param_Len & 0xff;
    buf[O_Param_Len + 5] = ack_out;
    for (uint16_t i = 0; i < (O_Param_Len + 6); i++)
        crc = _crc_xmodem_update(crc, buf[i]);
    buf[O_Param_Len + 6] = (crc >> 8) & 0xff;
    buf[O_Param_Len + 7] = crc & 0xff;
    buf_size = (O_Param_Len + 8);

    return buf_size;
}

uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data)
{
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}
