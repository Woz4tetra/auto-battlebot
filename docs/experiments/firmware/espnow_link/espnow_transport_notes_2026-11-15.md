# ESP-NOW transport notes

## Serial port DTR and RTS

Linux raises DTR and RTS together on open, and this never touches them. Changing them can reset an ESP32-S3 USB-Serial/JTAG device (the ESP-NOW dongle): DTR low with RTS high holds it in reset.

## CALIB_STAT packing

CALIB_STAT packs four 0-3 levels: sys in bits 7-6, then gyro, accel, mag.

## Transport latency sample

Unsigned subtraction wraps correctly across the 32-bit rollover. Each sample also holds the robot's wait for its next 10 ms telemetry slot, so the per-period minimum is the transport estimate.

## ACK rate

ACK rate over the last period. Only meaningful once robot_known: broadcast frames report success with no ACK behind them.
