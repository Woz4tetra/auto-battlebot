# README

## Notes moved from code comments

### INA238 wiring

- `mr_stabs_mk2/include/vbat_sensor.h`: Pack voltage and current from the main board's TI INA238 on the BNO055's I2C bus (Wire1). Board wiring (hardware/mr_stabs_mk2_pcb/pcba/design.py): - High side: switched pack + (BAT_IN) through the 0.5 mohm shunt R1 to VBATT, the ESC bus. Vin+ and Vin- are Kelvin taps on the shunt pads through 10 ohm filter resistors. VBUS sits on the pack side of the shunt, so it reads the shunt's drop high: at most 35 mV in a 70 A stall. - VS, A0 and A1 on +3V3, which sets address 0x45.

### IMU orientation

- `mr_stabs_mk2/src/updown_sensor.cpp`: The Mk2 board carries the BNO055 face down relative to the robot: rotate the chip frame 180 degrees about X (P4: X kept, Y and Z negated), so every output is in the robot frame and +z is up with the robot upright. The chip forgets this on reset, so it is rewritten on every begin().
