#pragma once

namespace auto_battlebot {

/**
 * Health the transmitter reports each cycle. Sampled once per control loop iteration and passed
 * along whole, so a new signal from the transmitter means one more field here rather than another
 * accessor threaded through the control loop, the runner, and the UI.
 *
 * `connected` is transport-level: the serial port or socket is open. `receiving_channels` is the
 * stream on top of it. The two differ on OpenTX, where the port stays open while the radio sends
 * nothing -- trainer mode failing to engage looks exactly like that. A transmitter that cannot
 * tell the two apart reports the same value for both.
 */
struct TransmitterStatus {
    bool connected = false;
    bool receiving_channels = false;
};

}  // namespace auto_battlebot
