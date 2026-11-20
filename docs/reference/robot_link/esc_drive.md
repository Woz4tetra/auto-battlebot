# ESC drive

How `firmware/mr_stabs_mk2` maps commands to ESC duty, decodes ESC replies, and re-enters ESC tuning mode.

## Command to duty

- A command inside `stop_threshold` runs the ESC at duty 0. Any other command runs `esc_map`'s floor-offset duty, so a 1% command runs about 7%.
- The ESCs run every step from 1 to 65 at the same speed, the speed of step 65. They respond linearly from step 66 up.
- Commands start at step 66, so the smallest command past `stop_threshold` already moves the motor.

## eRPM reply clock

- The ESC's reply clock is its own. BLHeli_32 on Mr Stabs sends bits at 95 ticks against the nominal 107, which leaves a 3-bit run at 2.66 bits, close to rounding down.
- `erpm_decode` measures the bit from the reply's single-bit runs (within 35% of nominal) and decodes with that.

## Tuning mode reentry

- Entry waits until COMBAT has lasted `REENTRY_MS` (`esc_config.h`).
- Exiting restarts the ESCs. Entering again right away left them out of the bootloader.

## Workarounds

- ESC UART receive: `esc_config.cpp` sets the RX full threshold to 1 so each byte is handed over as it lands. The default 120-byte threshold holds a reply back for 62 ms at a time, longer than the 10 ms gap that ends it.
- Shared UART pin: `uart_set_pin()` leaves the shared pin input only. Open drain gives the UART the line back. `gpio_set_direction()` reconnects the pin to plain GPIO output, so the UART's TX signal is routed to it again last. Unverified against IDF 4.4 source.
- A pending ESC config request is checked after the frame, so a write always finishes before the pins change hands. `EscConfig::update()` runs from `loop()` every pass and at most one queued frame, which blocks for up to about 3 s (a write). Armed and driving refuse entry, and a radio button press exits. The HTTP side is safe to call from the network task.
- DShot RMT channels: `DShot.cpp` declares NeoPixel's `rmt_reserved_channels[]` (defined in Adafruit NeoPixel `esp.c` on IDF below 5) and marks its own channels so NeoPixel skips them. pioarduino drops the array.
- DShot reply capture: the reply starts on a low run, but the capture can open with a stray edge or two. The neighbouring ESC's frame is often still on the wire when this receiver arms, and a reception that began before our own frame holds it ahead of a long gap. The decoder tries the first few low runs after any gap, and the GCR and CRC reject a wrong start. Unverified.
- `blheli_boot` reads replies until the line goes quiet and checks them from their end. Switching the pin from transmit to receive sometimes decodes as a stray byte (0xfe) ahead of the reply, seen on the robot on 2026-10-23. The passthrough's first-byte checks would fail on it the same way. No recording is in the repo.
- A bootloader read changes nothing, so it repeats. On the robot a session's first 256-byte read failed its CRC three times running and took the session down (2 CLI runs in 9 on 2026-10-23), so the last retry redoes the handshake before reading again. The root cause is not identified.
- `DEVICE_RESET` only selects the ESC. BLHeliSuite and the clients send it before each DeviceInitFlash and after each session. A real restart holds the line low, so the ESC boots its application and has to fall back into the bootloader through its lost-signal check. A handshake landing on that check left the ESC unreachable in 1 session in 10 on 2026-10-23. The ESC is already in its bootloader, so only select it. Leaving tuning mode, the idle timeout, and a failed handshake still restart it with `reset_esc()`.
