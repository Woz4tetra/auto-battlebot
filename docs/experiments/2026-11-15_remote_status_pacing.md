# Remote status pacing

- `/status/app` repeats every 5 s so a relay that connects late still gets it.
- `/status/network` is paced by `HostServices` itself (after each change and every 5 s), and `/status/battery` by `BatteryMonitor` (once a second), so the status bus does not throttle either.
- `/status/camera_settings` is published right after a camera command, otherwise twice a second so the auto-exposure read-back stays current.
