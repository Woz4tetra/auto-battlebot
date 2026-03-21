# BlHeli-Passthrough

Arduino Library and Examples for BlHeli Passtrough with RP2040, ESP32 or Atmega328P (Arduino UNO, Pro Mini, ...)

Sketch for BlHeli Passthrough without Flight Controller Just for BlHeli32 or AM32 and all other imARM_BLB ESC's

## ESP32

-   PlatformIO target board: `adafruit_qtpy_esp32s3_n4r2`
-   Default ESC signal pins: `A2` (GPIO9) and `A3` (GPIO8)
-   ESC channel 0 maps to `A2`, channel 1 maps to `A3`
-   USB serial passthrough is the default transport (BLE path is not used in this build)

### Build and upload (PlatformIO)

```bash
cd firmware/BlHeli-Passthrough
pio run
pio run -t upload
```

### needed Libraries for ESP32 use

-   Espressif ESP32: https://github.com/espressif/arduino-esp32
-   ESPSoftwareserial: https://github.com/plerup/espsoftwareserial/

all Libraries are available in Arduino Library Manager

## RP2040

-   You can set up to 8 pins for 8 ESCs, the usage is documented in rp2040.ino
-   I found [this](https://arduino-pico.readthedocs.io/en/latest/install.html) core to be the best for the RP2040, but others will likely work too without modification
-   No external libraries needed

## Atmega328P

-   Default Pins: Servo RX = GPIO11; Servo TX = GPIO 10
-   connect Servo Signal to Servo RX (11)
-   connect 1k Resistor between Servo RX (11) and Servo TX (10)

-   Please edit the #define ESC_RX and ESC_TX in ESC_Serial.cpp to choose other Pins

You have to Change SoftwareSerial RX Buffer Size in:
- Windows: C:\Users\User\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.6.23\libraries\SoftwareSerial\src
- MaxOS: /Users/{username}/Library/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/SoftwareSerial/src 

Change #define _SS_MAX_RX_BUFF 64 to 300 in SoftwareSerial.h

BlHeli Configurator Firmware Flashing doesn't work (Keep Alive Bug in Configurator) -> should be fixed already

## ESC

### BlHeli_32 https://github.com/bitdump/BLHeli/tree/master/BLHeli_32%20ARM

-   BlHeliSuite32: https://drive.google.com/drive/folders/1Y1bUMnRRolmMD_lezL0FYd3aMBrNzCig
-   BlHeli32 Android App: https://play.google.com/store/apps/details?id=org.blheli.BLHeli_32

### AlkaMotors_32 https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware

-   ESCConfigTool: https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/tree/master/Release/CONFIG%20TOOL
-   BlHeliConfigurator 1.3.0: https://drive.google.com/file/d/16N_l4Ukb4IBh8jnZvUq8hfmHY9Nbazs3/view?usp=sharing
