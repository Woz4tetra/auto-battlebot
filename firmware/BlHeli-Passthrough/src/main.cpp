#include <Arduino.h>
#include "serial_comm.h"

void setup()
{
    // A 4-way write frame is 263 bytes. The USB-Serial/JTAG driver buffers 256 by default and
    // drops the rest, which fails every flash write on a CRC error.
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
}

void loop()
{
    process_serial();
}
