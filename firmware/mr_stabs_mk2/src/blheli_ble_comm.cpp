#include <Arduino.h>
#include <NimBLEDevice.h>
#include "blheli_ble_comm.h"
#include "blheli_msp.h"
#include "blheli_4way.h"
#include "blheli_esc_serial.h"

static NimBLEServer *pServer = nullptr;
static BLECharacteristic *BlHeli_Read_CHR = nullptr;
static BLECharacteristic *BlHeli_Write_CHR = nullptr;

static bool ble_command = false;
static bool ble_connected = false;
static bool ble_disconnected = false;
static uint16_t ble_buffer_len = 0;
static volatile uint16_t ble_rx_counter = 0;
static volatile uint16_t true_ble_rx_counter = 0;
static uint16_t ble_tx_counter = 0;
static uint8_t tx_counter = 0;
static uint8_t ble_rx[5001];

class PassthroughServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
    {
        Serial.println("BLHeli: BLE connected");
        pServer->updateConnParams(desc->conn_handle, 6, 24, 0, 400);
        ble_rx_counter = 0;
        ble_tx_counter = 0;
        ble_connected = true;
    }

    void onDisconnect(NimBLEServer *pServer)
    {
        Serial.println("BLHeli: BLE disconnected");
        ble_rx_counter = 0;
        ble_tx_counter = 0;
        ble_disconnected = true;
        ble_connected = false;
    }
};

class PassthroughWriteCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic)
    {
        std::string data = pCharacteristic->getValue();
        true_ble_rx_counter = data.length();
        for (uint16_t i = 0; i < data.length(); i++)
        {
            ble_rx[ble_rx_counter] = data[i];
            if (ble_rx_counter >= 5000)
                ble_rx_counter = 0;
            else
                ble_rx_counter++;
        }
        ble_command = true;
    }
};

void blheli_ble_init()
{
    NimBLEDevice::init("MR-STABS-ESC");
    NimBLEDevice::setMTU(517);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new PassthroughServerCallbacks());

    NimBLEService *pService = pServer->createService("1000");
    BlHeli_Read_CHR = pService->createCharacteristic("1002", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    BlHeli_Write_CHR = pService->createCharacteristic("1001", NIMBLE_PROPERTY::WRITE);
    BlHeli_Write_CHR->setCallbacks(new PassthroughWriteCallback());
    pServer->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("MR-STABS-ESC");
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->start();

    ble_command = false;
    ble_connected = false;
    ble_disconnected = false;
    ble_rx_counter = 0;
    ble_tx_counter = 0;
    tx_counter = 0;

    Serial.println("BLHeli: BLE passthrough ready");
}

void blheli_ble_deinit()
{
    if (pServer)
    {
        NimBLEDevice::getAdvertising()->stop();
        NimBLEDevice::deinit(true);
        pServer = nullptr;
        BlHeli_Read_CHR = nullptr;
        BlHeli_Write_CHR = nullptr;
    }
    ble_command = false;
    ble_connected = false;
    ble_disconnected = false;
    ble_rx_counter = 0;
    ble_tx_counter = 0;
    tx_counter = 0;
}

bool blheli_ble_is_connected()
{
    return ble_connected;
}

void blheli_ble_process()
{
    if (ble_command)
    {
        ble_command = false;
        delay(5);
        uint16_t peer_mtu = pServer ? (pServer->getPeerMTU(0)) - 3 : 20;
        if (peer_mtu < 20 || peer_mtu > 517)
            peer_mtu = 20;

        if (ble_rx[4] == 0)
            ble_buffer_len = 256 + 7;
        else
            ble_buffer_len = ble_rx[4] + 7;

        if ((ble_rx[0] == 0x2F) && ((ble_rx_counter == ble_buffer_len) || (true_ble_rx_counter == ble_buffer_len)))
        {
            // 4-Way command
            ble_rx_counter = 0;
            ble_tx_counter = Check_4Way(ble_rx);
            do
            {
                if (ble_tx_counter <= peer_mtu)
                {
                    BlHeli_Read_CHR->setValue(&ble_rx[tx_counter], ble_tx_counter);
                    BlHeli_Read_CHR->notify(true);
                    ble_tx_counter = 0;
                }
                else
                {
                    BlHeli_Read_CHR->setValue(&ble_rx[tx_counter], peer_mtu);
                    BlHeli_Read_CHR->notify(true);
                    tx_counter = tx_counter + peer_mtu;
                    ble_tx_counter = ble_tx_counter - peer_mtu;
                }
                delay(50);
            } while (ble_tx_counter > 0);
            ble_tx_counter = 0;
            ble_rx_counter = 0;
            tx_counter = 0;
        }
        else if (ble_rx[0] == 0x2F)
        {
            // Incomplete 4-Way command, wait for more data
            ble_command = false;
        }
        else if (ble_rx[0] == 0x24 && ble_rx[1] == 0x4D && ble_rx[2] == 0x3C)
        {
            // MSP command
            ble_tx_counter = MSP_Check(ble_rx, ble_rx_counter);
            do
            {
                if (ble_tx_counter <= peer_mtu)
                {
                    BlHeli_Read_CHR->setValue(&ble_rx[tx_counter], ble_tx_counter);
                    BlHeli_Read_CHR->notify(true);
                    ble_tx_counter = 0;
                }
                else
                {
                    BlHeli_Read_CHR->setValue(&ble_rx[tx_counter], peer_mtu);
                    BlHeli_Read_CHR->notify(true);
                    tx_counter = tx_counter + peer_mtu;
                    ble_tx_counter = ble_tx_counter - peer_mtu;
                }
            } while (ble_tx_counter > 0);
            ble_tx_counter = 0;
            ble_rx_counter = 0;
            tx_counter = 0;
        }
        else
        {
            ble_tx_counter = 0;
            ble_rx_counter = 0;
            tx_counter = 0;
        }
        delay(10);
    }

    if (ble_disconnected)
    {
        blheli_esc_serial_end();
        ble_disconnected = false;
    }
}
