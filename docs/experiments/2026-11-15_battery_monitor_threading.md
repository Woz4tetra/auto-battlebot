# Battery monitor threading

`BatteryMonitor` reads the robot computer's battery on its own thread and publishes `/status/battery`. Each period it takes one sample, steps the estimator, logs the `battery_ina219` diagnostics, and publishes.

- An I2C read can take tens of milliseconds, which is why it never runs on the main loop.
- Construct it after `DiagnosticsLogger::initialize`, or the diagnostics are dropped.
