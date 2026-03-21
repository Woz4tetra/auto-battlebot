#include <Arduino.h>
#include "serial_comm.h"

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    process_serial();
}
