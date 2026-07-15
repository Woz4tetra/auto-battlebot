# Velocity Jig Wiring Sheet

Jig to measure a robot's angular and linear velocity separately.

**Host:** Feather RP2040 Adalogger (PID 5980)
**Buses:** I2C1 (OLED) - SPI1 (IMU) - SPI0 (onboard SD, isolated)

## Parts

| Part | PID | Interface |
|---|---|---|
| Feather RP2040 Adalogger (8MB flash + microSD) | 5980 | host |
| ISM330DHCX 6-DoF IMU | 4502 | SPI |
| 128x64 OLED FeatherWing (SH1107) | 4650 | I2C, stacked |
| LiPo 3.7V 420mAh | 4236 | JST PH2 |
| JST 2-pin extension cable with on/off switch | 3064 | inline power |

## Bus map

| Bus | GPIO | Peripheral | Address / CS |
|---|---|---|---|
| I2C1 | 2 / 3 | OLED SH1107 | `0x3C` |
| SPI1 | 14 / 15 / 8 | ISM330DHCX | CS = GPIO10 |
| SPI0 | 16 / 18 / 19 / 20 / 23 | microSD (onboard) | off-limits |

The IMU (SPI1) and SD card (SPI0) sit on separate SPI controllers, so high-rate
logging and IMU reads never contend for a bus.

## Master wiring table

| Feather pin | GPIO | Wire to | Function |
|---|---|---|---|
| **IMU - ISM330DHCX (SPI1)** | | | |
| SCK | 14 | IMU **SCL** | SPI clock (SCL pad = SCK in SPI) |
| MO | 15 | IMU **SDA** | MOSI (SDA pad = SDI in SPI) |
| MI | 8 | IMU **DO** (bottom) | MISO |
| D10 | 10 | IMU **CS** (bottom) | chip select, low = SPI |
| D12 | 12 | IMU **I1** | INT1 data-ready |
| 3V | - | IMU **VIN** | power |
| GND | - | IMU **GND** | ground |
| **OLED FeatherWing (I2C1, stacked)** | | | |
| SDA | 2 | OLED SDA | I2C data |
| SCL | 3 | OLED SCL | I2C clock |
| D9 | 9 | Button A | input pull-up |
| D6 | 6 | Button B | input pull-up |
| D5 | 5 | Button C | input pull-up |
| RST | - | OLED Reset | board reset line, no GPIO |
| 3V / GND | - | OLED 3V / GND | power |
| **Encoder (reserved, quadrature-ready)** | | | |
| A0 | 26 | Encoder **A** | quad A, or unused if single |
| A1 | 27 | Encoder **B** | quad B, or single-ch + PWM count |
| 3V | - | Encoder **Vcc** | see power note |
| GND | - | Encoder **GND** | ground |

## Power

Chain: LiPo -> PID 3064 switch cable -> Feather JST. The switch cuts battery
power only. USB overrides and charges the cell when plugged in.

## Spare GPIOs

`D11 (11)` - `D13 (13, LED)` - `TX/RX (0/1)` - `A2/A3 (28/29)` - `D24/D25 (24/25 if present)`.

A2/A3 is a second consecutive pair, so a second quadrature encoder is possible.

## Off-limits

GPIO **16, 18, 19, 20, 23**, consumed by the onboard microSD (SPI0).

## Interrupt capability

Every RP2040 GPIO (0-29) supports hardware interrupts with four selectable
triggers each: rising edge, falling edge, level-high, level-low. They feed the
`IO_IRQ_BANK0` controller, demultiplexed per pin by the SDK. There is no scarce
INT0/INT1 pin like on AVR.

Confirmed for the pins used here:

| Pin | GPIO | Interrupt use | Capable |
|---|---|---|---|
| D12 | 12 | IMU INT1 data-ready | Yes |
| A0 | 26 | encoder A | Yes |
| A1 | 27 | encoder B | Yes |

A0/A1 are ordinary digital GPIOs with the ADC mux on top; as digital inputs they
interrupt like any other pin.

Three independent hardware paths exist for a pulse or encoder signal:

1. **GPIO edge interrupt** - ISR on any pin. Universal.
2. **PWM B-input edge counter** - zero-CPU pulse counting, only on B-channel
   pins (odd GPIOs such as 27). Single-channel use.
3. **PIO state machine** - zero-CPU quadrature decode; reads pins directly, does
   not use the interrupt controller. Needs the two channels on consecutive GPIOs
   (A0/A1 qualify). 8 state machines available.

For quadrature, use path 3 (PIO). Path 1 is the fallback and is available on both
encoder pins.

Applies to the arduino-pico core (Earle Philhower) that Adafruit's RP2040 board
support uses, and to CircuitPython/MicroPython. Do not build against the older
Arduino Mbed RP2040 core. Reference: RP2040 datasheet section 2.19.6 (IO_BANK0
interrupts).

## Notes

1. **SPI mode drops STEMMA QT for the IMU.** Solder the six IMU wires to the
   breakout header; the Qwiic port is I2C only.
2. **IMU silk uses I2C names.** There is no `SCK` pad. On the primary (bottom)
   row, in SPI mode `SCL`=SCK, `SDA`=MOSI/SDI, `DO`=MISO, `CS`=chip select,
   `I1`/`I2`=INT1/INT2. Leave the entire AUX top row (`SCX SDX CS DO GND`)
   unconnected; it is a secondary bus for chaining another sensor, not an
   alternate way to reach the IMU.
3. **Stack the OLED with a Doubler/Tripler or stacking headers** so GPIO14/15/8,
   D10, D12, and A0/A1 stay reachable. A bare stack buries them.
4. **Encoder power:** on battery the Feather provides only 3.3V (3V pin) and
   ~3.7V (BAT). There is no 5V rail unless USB is connected. If the encoder needs
   5V, run it off USB or add a boost converter. Wire to 3V only if it accepts 3.3V.
5. **No I2C address conflict:** the IMU is off I2C entirely; the OLED owns the bus
   at `0x3C`.
