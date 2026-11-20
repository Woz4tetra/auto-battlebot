# OpenTX transmitter

`src/transmitter/opentx_transmitter.cpp` talks to the radio over USB serial (found by VID/PID). `Esp32DiagnosticsOpenTxTransmitter` and the ESP-NOW transmitter share its port handling.

## Workarounds

- `enable()` and `disable()` gate only the trainer command output. The telemetry streams stay on whenever the port is open, so link statistics and the init button on the radio keep working with autonomy disabled.
- `reconnect_if_needed` closes a port that is open but has had no channel frames for `kChannelTimeout`. Trainer mode dropped out or the `channels on` stream never started, and re-priming needs a fresh open. It is rate limited by `next_reconnect_attempt_` so a radio that never streams is retried once per interval instead of thrashing the port every tick. Unverified: radio firmware behaviour, not checkable here.
