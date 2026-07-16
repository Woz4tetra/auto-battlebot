// TEMP hardware bring-up test for the velocity jig board.
//
// Blinks the onboard LED (GPIO13) fast and prints a counter over USB serial.
// Neither depends on the OLED, IMU, or SD, so this isolates the board itself:
//   - LED blinking      -> upload works and code is running
//   - counter on ttyACM -> USB CDC serial works (so ./monitor would too)
//
// Delete firmware/blink_test/ when done.

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    static uint32_t n = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
    Serial.print("blink test alive: ");
    Serial.println(n++);
}
