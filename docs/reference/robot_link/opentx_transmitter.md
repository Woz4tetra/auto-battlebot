# OpenTX transmitter

`src/transmitter/opentx_transmitter.cpp` talks to the radio over USB serial (found by VID/PID). `Esp32DiagnosticsOpenTxTransmitter` and the ESP-NOW transmitter share its port handling.

## Workarounds

- `initialize()` writes `telemetry on` and `channels on` every time the port (re)opens. Without it, a reconnect after an unplug, or a radio not yet present at startup, leaves the radio silent on the receive side until `enable()` happens to be called while the port is open.
- `last_channel_time_` is set at open. The radio gets a grace window to start streaming before the channel stream reads as stalled, so a healthy open does not flash a trainer-mode fault on the UI.
- `enable()` and `disable()` gate only the trainer command output. The telemetry streams stay on whenever the port is open, so link statistics and the init button on the radio keep working with autonomy disabled.
- `disable()` zeroes `prev_linear_command_` and resets `last_send_time_`, so re-enabling ramps up from standstill instead of resuming the last command sent before autonomy was cut.
- `log_switch_states` stamps the section on every frame, not only on change. The section's age then means the age of the last frame read.
- `reconnect_if_needed` closes a port that is open but has had no channel frames for `kChannelTimeout`. Trainer mode dropped out or the `channels on` stream never started, and re-priming needs a fresh open. It is rate limited by `next_reconnect_attempt_` so a radio that never streams is retried once per interval instead of thrashing the port every tick. Unverified: radio firmware behaviour, not checkable here.
- `EspNowTransmitter` applies the same rule to the dongle: a port open but quiet for `kDongleSilence` is closed and reopened, rate limited like open itself. A port whose device went away reads empty forever, so only a fresh open recovers it.
- `Esp32DiagnosticsOpenTxTransmitter::~Esp32DiagnosticsOpenTxTransmitter` calls `diagnostics_->stop()` first so the worker is gone before the channel it feeds.
- None of this has a test. `serial_` is a concrete `SerialPort` with no injection point, and the transmitter needs a process-global `DiagnosticsLogger`. A loopback `SerialPort` fake would cover the open, enable, disable and stall rows.
