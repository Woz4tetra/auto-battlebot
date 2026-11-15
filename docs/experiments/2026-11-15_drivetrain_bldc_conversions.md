# Drivetrain BLDC conversions

## Drivetrain BLDC conversions

### `playground/bldc_conversions/bldc_max_rpm.py`

`bldc_max_rpm.py`:

Theoretical max RPM calculator for the drivetrain BLDC motors.

```
Hardware:
- 2x Repeat Compact 1806, 2300 Kv, 22.6:1 gearbox
  https://repeat-robotics.com/products/repeat-compact-1806
- 2x BLHeli ESC
- 2x GNB 2S 300mAh LiHV battery wired in series -> 4S bus
  https://www.racedayquads.com/products/gaoneng-gnb-3-8v-2s-300mah-80c-lihv-whoop-micro-battery-w-cabled-connector-xt30
```

Theoretical no-load speed is just Kv times the applied voltage. Kv is the back-EMF constant: at N rpm the motor generates N/Kv volts of back-EMF, so the rotor spins up until back-EMF equals the supply and no more current flows. Real top speed is a few percent lower (winding resistance, iron/friction losses, ESC timing), but this is the ceiling the ESC can command.

```
The 2300 Kv figure is validated by the manufacturer's own output-shaft numbers:
  2300 * 14.8 V (4S nominal) / 22.6 = 1506 rpm  (spec sheet says 1500 rpm)
  2300 * 11.1 V (3S nominal) / 22.6 = 1130 rpm  (spec sheet says 1130 rpm)
```

```
Usage:
    python playground/bldc_max_rpm.py
```

- `CELL_NOMINAL_V`: GNB LiHV cell. LiHV charges to 4.35 V/cell (vs 4.20 for a plain LiPo) and sags to ~3.8 V/cell nominal.

### `playground/bldc_conversions/rpm_accel_convert.py`

`rpm_accel_convert.py`:

Convert between output-shaft acceleration (RPM/s) and wheel-surface linear acceleration (m/s^2).

This is the same relation the OpenTxTransmitter uses to turn its `max_motor_rpm_per_sec` limit into a physical acceleration cap.

```
The wheel contacts the ground at radius r = wheel_diameter / 2. A shaft turning
at N rpm moves the contact patch at:
```

```
    v = omega * r = (N / 60 * 2*pi) * (wheel_diameter / 2)
      = N * pi * wheel_diameter / 60      [m/s]
```

Differentiating (wheel diameter is constant) gives the acceleration pair:

```
    a       = dRPM/dt * pi * wheel_diameter / 60      [m/s^2]
    dRPM/dt = a * 60 / (pi * wheel_diameter)          [RPM/s]
```

Note this is the OUTPUT-shaft rpm (after the gearbox), which is what `max_motor_rpm` / `max_motor_rpm_per_sec` in the transmitter config refer to.

The measured slip/flip ceiling for this robot is 8.5 m/s^2. At the config default wheel_diameter of 0.05 m that works out to ~3247 RPM/s.

```
Usage:
    python playground/rpm_accel_convert.py                 # reference table
    python playground/rpm_accel_convert.py --mps2 8.5      # m/s^2  -> RPM/s
    python playground/rpm_accel_convert.py --rpm-per-sec 3000  # RPM/s -> m/s^2
    python playground/rpm_accel_convert.py --mps2 8.5 --wheel-diameter 0.06
```

- `DEFAULT_WHEEL_DIAMETER_M`: output-shaft wheel diameter, meters measured linear accel before slipping/flipping.
