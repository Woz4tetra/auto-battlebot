# Plan: ESP-NOW command and telemetry link for mr_stabs_mk2

## Goal

Add a direct ESP-NOW link between the Jetson and the mr_stabs_mk2 firmware that:

1. Delivers autonomous velocity commands with roughly 5 ms less transport delay.
2. Streams the firmware diagnostics struct back to the Jetson at loop rate.
3. Lets the on-robot web dashboard be deleted, freeing RAM and flash for the camera port.

The Crossfire link keeps full authority. ESP-NOW is a transport shortcut layered on top of the
existing path, never a replacement for it.

### Standing decision: latency wins every tradeoff

Where a choice in this plan trades latency against something else, take the lower-latency option.
Applied throughout, that resolves to:

| Choice | Decision | Cost accepted |
|---|---|---|
| ESP-NOW PHY rate | 6 Mbps OFDM via `esp_now_set_peer_rate_config()` | No `WIFI_PROTOCOL_LR`, so ~10 dB less link budget |
| Channel | Dongle scans and picks at boot, robot sweeps to find it | Each match runs on an untested channel |
| Dongle repeat rate | 250 Hz, forwarding on arrival | ~6% channel duty cycle |
| `FRESH_STREAK_REQUIRED` | 3, not 5 | Slightly more source-switch flapping at range |
| CRSF serial baud | 921600, not the 400000 default | None, it is free |
| Combat-mode SoftAP | Off | No dashboard during a match |

Each is justified in the section that raises it. The range cost of dropping LR is the one that
carries real risk, and it is mitigated by the fact that losing ESP-NOW costs latency, not control.

## Tradeoffs

### The radio configuration is already optimal

Every setting that could have been a cheap win has been checked and is already at its best value:

| Parameter | Status | Evidence |
|---|---|---|
| Crossfire over-air rate | 150 Hz, Crossfire's maximum | Confirmed by operator |
| EdgeTX mixer scheduler | Enabled, module-synchronized | `radio/src/targets/taranis/hal.h:2961` (x9dp2019), `horus/hal.h:1212` (tx16s) |
| Mixer period source | Driven by the Crossfire module's reported rate | `radio/src/pulses/crossfire.cpp:191-199` |
| CRSF serial baud | 400000 minimum, 4 ms frame period | `radio/src/telemetry/crossfire.h:126-146` |
| 115200 baud (16 ms period) | Unreachable on this hardware | `crossfire.h:136`, `CROSSFIRE_MAX_EXTERNAL_BAUDRATE` on STM32F4 excludes it |

There is no free latency left in the radio config. Any further reduction requires a different
link, which is what this plan proposes.

### Latency: fixed pipeline delay

Two stages are common to both paths and cancel in the difference: the 33.3 ms command generation
interval (`camera_fps = 30` in `config/_common.toml:13`, with `send()` called once per tick at
`src/runner.cpp:473`) and the firmware `loop()` quantization.

**Current path, Jetson to firmware:**

| Stage | Best | Typical | Worst |
|---|---|---|---|
| Jetson to handset, USB CDC | 0.3 | 1.0 | 2.0 |
| Mixer quantization, synchronized to the 150 Hz module | 0 | 3.3 | 6.7 |
| Handset to module, CRSF 26 B at 420 kbaud | 0.62 | 0.62 | 0.62 |
| Module processing plus air time | 1.0 | 1.5 | 2.5 |
| RX processing plus UART out | 0.9 | 1.2 | 1.6 |
| **Total (ms)** | **2.8** | **7.6** | **13.4** |
| One lost frame costs a full period | | | **+6.7 = 20.1** |

**ESP-NOW path, and it depends heavily on the PHY rate:**

| Stage | 6 Mbps OFDM | 1 Mbps DSSS (default) | LR mode |
|---|---|---|---|
| Jetson to dongle, USB CDC | 1.0 | 1.0 | 1.0 |
| Dongle parse plus `esp_now_send` queue | 0.6 | 0.6 | 0.6 |
| DIFS, backoff, data, SIFS, ACK | 0.25 | 1.4 | ~3.0 |
| Recv callback to shared struct | 0.2 | 0.2 | 0.2 |
| **Typical total (ms)** | **2.1** | **3.2** | **4.8** |

**Resulting saving:**

| Case | Saving |
|---|---|
| Best case, both clean | ~2 ms |
| Typical, ESP-NOW at 6 Mbps OFDM | ~5.5 ms |
| Typical, ESP-NOW at the 1 Mbps default | ~4.4 ms |
| Typical, ESP-NOW with `WIFI_PROTOCOL_LR` enabled | ~2.8 ms |
| Crossfire loses a frame, ESP-NOW clean | ~15 ms |
| ESP-NOW hits a retry chain, Crossfire clean | **ESP-NOW is 5-15 ms worse** |

These are computed from frame sizes, baud rates, and 802.11 timing constants. Nothing here is
measured. Phase 1 measures it, and the `jetson_tx_us` round-trip echo gives it directly.

### Range and latency pull in opposite directions

`WIFI_PROTOCOL_LR` buys roughly 10 dB of link budget by lowering the PHY rate, which roughly
doubles air time and eats about half the latency saving. You can have the range mitigation or the
fastest link, not both.

**Decision: 6 Mbps OFDM, no LR.** That is the ~5.5 ms row rather than the ~2.8 ms row. Two things
make the range cost tolerable: the trainer path still carries every command, so a weak ESP-NOW
link degrades to today's latency rather than to no control; and the failure is measurable in
Phase 1 as ACK rate, so it will not be a surprise.

If arena testing shows the link failing on range rather than congestion, the fallback is not LR
but a better antenna and dongle placement, which costs no latency.

### Channel selection is a latency lever

CSMA backoff sits directly in the ESP-NOW budget, and backoff grows with channel occupancy. A
congested channel does not just raise loss, it raises median latency through deferral and retry.

**Decision: the dongle picks the channel at boot and the robot sweeps to find it.** No pit scan,
no per-event constant edit, nothing to do on competition day.

The obvious version of this breaks. If both ends scan independently and each picks the quietest
channel, they will disagree: different antennas, different locations (dongle at the Jetson, robot
in the box), different instants, different tie-breaks. A channel mismatch produces silent send
failures, not an error, so the link would be dead with nothing in the logs explaining why.

Make it asymmetric instead.

**The dongle picks.** One scan at boot, then fixed for the rest of the run. The dongle is the right
authority because it is not the end that takes hits and brown-out reboots mid-match. Score all 11
channels by occupancy but only ever select from {1, 6, 11}, so the pick absorbs adjacent-channel
energy from its neighbors instead of landing in the skirts between the clusters.

**The robot discovers by sweeping.** The dongle already transmits every 4 ms (the 250 Hz repeat
below), which gives the robot a continuous signal to search for:

```
for channel in [last_known_good, 1, 6, 11]:
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE)
    wait 25 ms                     // ~6 dongle repeats plus switch settling
    if a frame carrying the session magic arrived:
        lock, persist the channel, done
```

Three candidates at 25 ms is a 75 ms worst-case sweep. The common case is one dwell on the
last-known-good channel, so 25 ms. The robot runs on the trainer path for the duration, which is
the same degradation as any other ESP-NOW dropout.

The sweep also covers re-acquisition. After `REACQUIRE_SILENCE_MS` with no valid frame, the robot
resumes sweeping. That self-heals a robot reboot mid-match, which a flashed constant does not, and
it self-heals the dongle restarting on a different channel.

#### What auto selection costs

- **The scan measures beacons, not airtime.** The interferers that actually drive CSMA backoff at
  an event are other teams' 2.4 GHz control links, video TX, Bluetooth, and the ZED's own USB3
  noise. None of them emit beacons, so an AP-count scan can rank a channel quiet while it is
  saturated. Sampling occupancy in promiscuous mode is the better metric and costs 100 ms or more
  per channel, so budget 1-3s of dongle boot time for a scan worth trusting.
- **Boot time and boot location are the wrong sample.** The dongle scans where the Jetson powers
  on, not in the cage, and before the match rather than during it. The pit scan has the same flaw,
  so this is not a regression, but auto selection does not fix it either.
- **Reproducibility is gone.** Every match runs on a channel that was never bench tested. Mitigate
  by reporting the selected channel in `diag_data_t` and the `0x04` link stats frame so it lands in
  the MCAP, and by pinning `ESPNOW_CHANNEL_OVERRIDE` for bench runs.
- **Sweep flap.** A weak but alive link can cross the staleness threshold and trigger a sweep that
  loses the packets which would have recovered it. `REACQUIRE_SILENCE_MS` sits well above
  `COMMAND_STALE_MS` for exactly this reason.
- **No rescan mid-run.** A dongle that periodically rescans to stay adaptive drops the link on
  every rescan while the robot re-acquires. Scan once at boot. The only other scan trigger is the
  explicit combat to tuning transition.

What makes this an acceptable trade is that ESP-NOW holds no authority. A bad pick costs ~5 ms of
latency, not control, because the trainer path carries every command. If ESP-NOW were the control
link, a flashed constant and a pit scan would be the right answer.

### The 30 Hz command rate dominates everything, and is solved elsewhere

At 30 fps a new command exists only every 33.3 ms, so the average command is already 16.7 ms stale
before any transport touches it. That is more than three times what this entire plan saves.

**Perception cannot be sped up.** The camera runs at 30 fps (`config/_common.toml:13`) and the
TensorRT inference behind it sets the rest. Raising the frame rate is not the fix.

**This is solved by [`control_loop_thread_plan.md`](control_loop_thread_plan.md)**, which moves the
filter, target selection, navigation, and transmit onto their own thread at a configurable rate
(default 250 Hz). Perception becomes a slow measurement correcting a fast prediction rather than
the clock gating the pipeline. That plan claims ~15 ms from the command interval alone, and up to
~66 ms once the filter predicts forward to compensate for perception latency.

Two consequences for this plan:

1. **The Jetson will send at the control rate, not 30 Hz.** The dongle repeat below becomes a
   safety net covering real packet loss rather than a correctness requirement covering a slow
   producer.
2. **`CompositeTransmitter` needs a per-child transmit rate cap.** The trainer path cannot absorb
   250 Hz; ESP-NOW can. Both plans need this, and it belongs in the composite.

Sequencing: the control loop thread is worth roughly 13x what this plan's command path is worth and
needs no new hardware. Land it first.

### The trainer path is worth speeding up too

Both paths carry every command, and ESP-NOW falls back to the trainer path on any dropout, so the
fallback's latency is part of the real distribution rather than a rare edge case.

`CROSSFIRE_FRAME_PERIODS` in `radio/src/telemetry/crossfire.h:146` ties the frame period to the
CRSF serial baud: 400000 gives 4 ms, 921600 and above give 2 ms. Raising it also cuts the 26-byte
frame transmission from 0.62 ms to 0.28 ms.

**Decision: raise the model's CRSF baud to 921600.** The gain is roughly 1 ms and partly absorbed
when `getAdjustedRefreshRate()` has a valid module sync, but it costs nothing and it lowers the
fallback path. Set it in Model Setup on the mr_stabs_mk2 model; it is a per-model field
(`g_model.moduleData[EXTERNAL_MODULE].crsf.telemetryBaudrate`).

### Summary

| | Crossfire (current) | ESP-NOW |
|---|---|---|
| Typical transport delay | ~7.6 ms | ~2.1 to 4.8 ms |
| Frequency agility | FHSS across 868/915 MHz | None, one fixed 20 MHz channel |
| Band congestion at an event | Low | Severe |
| Behavior on a lost packet | Next frame in 6.7 ms, bounded | Exponential backoff, unbounded |
| Redundancy | Re-sends at 150 Hz regardless of new data | Only sends when the Jetson does, see below |
| Link quality metric | LQ and RSSI in CRSF telemetry | Must be derived from ACK counts |
| Telemetry bandwidth | A few hundred bytes/s | 72-byte struct at loop rate |
| Arm authority | Yes | Never |

ESP-NOW wins on median latency and telemetry bandwidth. Crossfire wins on every tail-behavior
property, which is why the design gives ESP-NOW no authority and always mirrors to the trainer
path.

### The dongle must re-send, or the tail gets worse

Crossfire transmits at 150 Hz continuously, re-sending the last trainer value whether or not the
Jetson updated it. A lost Crossfire frame is covered 6.7 ms later.

ESP-NOW as designed here sends only when `send()` is called, which is 30 Hz. One lost packet
leaves a 33.3 ms gap, and two consecutive losses exceed `COMMAND_STALE_MS` and drop back to the
trainer path.

Fix: the **dongle** repeats the last received command on its own 250 Hz timer and forwards
immediately on arrival rather than waiting for the next slot. That restores the redundancy
Crossfire gets for free and keeps the 60 ms staleness window meaningful.

Once `control_loop_thread_plan.md` lands, the Jetson sends at the control rate rather than 30 Hz,
so the repeat covers only real packet loss instead of also covering a slow producer. Build the
repeat anyway: it is the difference between a bounded and an unbounded dropout tail, and it keeps
this plan independent of the control loop work landing first.

### What this does not fix

ESP-NOW runs on the same 2.4 GHz radio as WiFi, on one fixed 20 MHz channel with no frequency
hopping. It is more exposed to arena congestion than Crossfire at 868/915 MHz, which is exactly
why it gets no authority.

## Design principle: authority and transport are separate

`radio_data_t` (`firmware/mr_stabs_mk2/include/crsf_bridge.h:20`) already splits the two concerns:

```c
typedef struct radio_data {
    float a_percent, b_percent;                 // transport: the command
    bool armed, connected, button_state;        // authority: arm, link state, mode
    three_state_switch_t flip_switch_state;
} radio_data_t;
```

ESP-NOW may substitute `a_percent` and `b_percent`. It may never touch `armed`, `connected`,
`button_state`, or `flip_switch_state`. Those stay on Crossfire so a switch throw on the handset
always stops the robot, including when the ESP-NOW link is the thing misbehaving.

### Both paths carry every command

The Jetson sends each command over the trainer link *and* over ESP-NOW. This gives the safest
degradation:

- ESP-NOW fresh: the firmware uses it, and the command arrives roughly 5 ms sooner.
- ESP-NOW stale or dead: the firmware falls back to the trainer channels, which carry the same
  autonomous command. Behavior degrades to exactly what `OpenTxTransmitter` does today.
- Trainer gate disengaged: the trainer channels carry the pilot's sticks, and the firmware's
  `flip_switch_state` gate blocks ESP-NOW substitution. Manual control is one switch throw with no
  code path involved.

### Bumpless transfer belongs in the firmware, not the Jetson

The two paths cannot be guaranteed bit-identical from the Jetson side. `limit_linear_acceleration`
(`src/transmitter/opentx_transmitter.cpp:128`) is stateful and reads
`get_channel_value(config_.trainer_enable_channel)` at line 134, so its behavior depends on handset
channel data that arrives only over the OpenTX serial link. A standalone `EspNowTransmitter` has no
channel input and cannot reproduce it. The limiter also sits behind the
`if (!serial_.is_open() || !enabled_) return;` guard at `opentx_transmitter.cpp:112`, so its state
stops advancing whenever that serial port drops, diverging from any parallel copy.

Rather than couple the two transmitters to keep their limiter state in sync, move the rate limit to
where the arbitration happens. The firmware slew-limits its arbitrated output, so whichever source
wins, the command ramps instead of stepping.

Applying the same rate limit twice in series is idempotent: when the Jetson already limited the
signal, the firmware limiter is a no-op and costs nothing. When the sources disagree, it catches
the step. This also makes the Jetson-side transmitters fully independent, which is what lets them
be separate implementations rather than one wrapping the other.

## Current state

### Firmware gates (all preserved)

`loop()` in `firmware/mr_stabs_mk2/src/main.cpp:234` runs three gates before driving motors:

| Line | Gate | Action on fail |
|---|---|---|
| `main.cpp:245` | `crsf->update()` returns `isLinkUp()` | `stop_escs()` |
| `main.cpp:49` | `COMMAND_TIMEOUT`, 5000 ms of identical frames | `stop_escs()` |
| `main.cpp:323` | `radio_data->armed` (CH4/SF) | `stop_escs()` |
| `main.cpp:377` | `is_upside_down` inverts `a_percent` | n/a |
| `main.cpp:387` | `mix_motor_outputs()` drives ESCs | n/a |

The ESP-NOW substitution goes after line 323. The new firmware slew limiter goes after line 377, so
it limits the post-inversion signal. Limiting before the flip would let a genuine inversion produce
a full-scale reversal in one tick.

Note on `COMMAND_TIMEOUT`: 5000 ms of identical frames is a backstop, not a watchdog.
`crsf->isLinkUp()` (`firmware/mr_stabs_mk2/src/crsf_bridge.cpp:15`) does the real failsafe work.
Do not copy the 5000 ms figure for the ESP-NOW staleness window.

### WiFi surface to be replaced

- `main.cpp:224` `WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)`, defaults to channel 1
- `main.cpp:225` `setup_ota()`, ArduinoOTA over TCP
- `main.cpp:229` `diag_server.begin(tunables)`, ESPAsyncWebServer plus AsyncTCP

`diag_data_t` (`firmware/mr_stabs_mk2/include/diagnostics_server.h:4`) packs to 72 bytes with
padding, well inside the 250-byte ESP-NOW payload.

Current `firmware.bin` is 866 KB.

### Jetson transmitter

`OpenTxTransmitter` (`src/transmitter/opentx_transmitter.cpp`) does two jobs:

1. **Send** (`opentx_transmitter.cpp:110`): slew-limits linear, runs `DifferentialDriveProcessor`,
   writes trainer channels.
2. **Read** (`opentx_transmitter.cpp:65`): parses CRSF telemetry and handset channels, which is
   where `did_init_button_press()` (`opentx_transmitter.cpp:186`), the `trainer_enable_channel`
   gate, and `CommandFeedback` all come from.

Job 2 has no ESP-NOW equivalent. `EspNowTransmitter` therefore implements only job 1 and reports
nothing on the read side, and a new `CompositeTransmitter` drives both.

`OpenTxTransmitter` needs no changes at all under this design.

## Architecture

Three pieces, each a plain `TransmitterInterface` implementation registered in the existing
factory (`src/transmitter/config.cpp:36`):

```
CompositeTransmitter                     fans send() out to every child
  ├── OpenTxTransmitter    (primary)     trainer channels + all read-side duties
  └── EspNowTransmitter                  framed serial to the USB dongle
```

`EspNowTransmitter` is standalone and usable on its own. That matters for bench work: you can run
it without a handset connected to drive the robot straight from the Jetson, which is not possible
with the current transmitter at all.

### Composite policy

The first child in config order is **primary**. Everything read-side comes from it.

| Method | Policy |
|---|---|
| `initialize()` | Call all. Return primary's result. Non-primary failures log a warning and are ignored. |
| `send()` | Fan out the raw command to every child. No preprocessing in the composite. |
| `update()` | Call every child so their receive paths drain. Merge feedback maps, primary applied last so it wins on key collision. |
| `did_init_button_press()` | Call every child (no short-circuit, so latches clear), return the OR. |
| `is_connected()` | Primary only. A dead dongle costs latency, not control. |
| `enable()` / `disable()` | Fan out to all. |

The composite does no command preprocessing. It is a pure fan-out, which keeps it reusable for any
future transmitter pairing.

## Wire protocol

Two hops with different framing. The Jetson never speaks ESP-NOW directly.

```
Jetson <--framed serial--> ESP32 dongle <--ESP-NOW unicast--> mr_stabs_mk2
```

### Serial framing (Jetson to dongle)

```
uint16_t magic   = 0xB07B
uint8_t  type
uint8_t  length            // payload bytes
uint8_t  payload[length]
uint16_t crc16             // CCITT over type, length, payload
```

Streaming-safe: the parser accumulates bytes and scans for magic, matching how `CrsfParser`
already handles partial reads from `SerialPort::read_available()`.

### Message types

| Type | Direction | Payload |
|---|---|---|
| `0x01` command | Jetson to robot | `seq:u32, jetson_tx_us:u32, linear:f32, angular:f32` |
| `0x02` telemetry | robot to Jetson | `diag_data_t` plus `seq:u32, jetson_tx_us:u32` echo |
| `0x03` tunable set | Jetson to robot | `id:u8, value:f32` |
| `0x04` link stats | dongle to Jetson | `sent:u32, acked:u32, failed:u32, last_rtt_us:u32, channel:u8` |

`0x04` originates on the dongle from `esp_now_register_send_cb`, not from the robot. It is the only
link-quality signal available, since ESP-NOW has no equivalent to Crossfire's LQ and RSSI.

Echoing `jetson_tx_us` in telemetry gives round-trip time directly. One-way is approximately half.

### Over-the-air frame (dongle to robot)

The dongle prepends a fixed 4-byte session magic to every ESP-NOW frame and drops any received
frame without it. Two teams running ESP-NOW example code on channel 1 is a realistic accidental
cross-talk failure, and 4 bytes rules it out. The Jetson does not need to know about this field.

Peer setup, with the gotchas that cause silent failures:

```c
esp_now_peer_info_t peer = {};
memcpy(peer.peer_addr, PEER_MAC, 6);
peer.ifidx   = WIFI_IF_AP;   // AP-only build: WIFI_IF_STA fails with ESP_ERR_ESPNOW_IF
peer.channel = 1;            // must match the SoftAP channel
peer.encrypt = false;
esp_now_add_peer(&peer);
```

Unicast to a hardcoded MAC gets MAC-layer ACK and retries. Broadcast gets neither.

Set the PHY rate explicitly. The 1 Mbps default costs ~1.2 ms per packet against 6 Mbps OFDM:

```c
esp_now_rate_config_t rate = {
    .phymode  = WIFI_PHY_MODE_11G,
    .rate     = WIFI_PHY_RATE_6M,
    .ersu     = false,
    .dcm      = false,
};
esp_now_set_peer_rate_config(PEER_MAC, &rate);
```

Do not enable `WIFI_PROTOCOL_LR` alongside this. The two are mutually exclusive and LR costs about
half the latency saving. See the standing decision in Goal.

### The dongle repeats the last command

Two distinct behaviors, and conflating them adds latency:

1. **Forward on arrival.** The moment a frame arrives from the Jetson, send it. Never wait for the
   next repeat slot, which would add up to one repeat period of pure delay.
2. **Repeat on timer at 250 Hz.** Independently, re-send the most recent command every 4 ms so a
   lost packet is covered quickly.

Without the repeat, a 30 Hz command stream means one lost packet leaves a 33.3 ms gap and two
consecutive losses trip `COMMAND_STALE_MS`. 250 Hz is chosen over 150 Hz to shorten post-loss
recovery; at ~80 bytes and 6 Mbps that is roughly 6% channel duty, which is affordable.

Reset the repeat only when a newer `seq` arrives. Never let the dongle invent or extrapolate a
command.

## Firmware changes

### New: `firmware/mr_stabs_mk2/include/espnow_link.h` and `src/espnow_link.cpp`

```c
namespace espnow_link {

struct command_t {
    uint32_t seq;
    uint32_t jetson_tx_us;
    float a_percent;
    float b_percent;
};

class EspNowLink {
   public:
    bool begin(uint8_t channel, const uint8_t peer_mac[6]);
    void end();                                  // for the combat/tuning mode transition

    /** True if a command newer than the last one arrived within COMMAND_STALE_MS. */
    bool fresh(uint32_t now_ms) const;
    const command_t &latest() const;

    void send_telemetry(const diag_data_t *diag);

   private:
    static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
};

// 15 missed packets at the dongle's 250 Hz repeat rate. Kept generous on purpose: falling back to
// the trainer path costs ~5 ms, so a window that drops out on transient loss works against the
// latency goal. Tighten only if stale commands prove to be a real hazard.
const uint32_t COMMAND_STALE_MS = 60;

// 3, not 5: at 250 Hz that is a 12 ms recovery instead of 20 ms. Re-enter the fast path sooner and
// accept slightly more switching at the edge of range, which the firmware slew limiter absorbs.
const uint8_t  FRESH_STREAK_REQUIRED = 3;

}  // namespace espnow_link
```

Rules the implementation must follow:

- **The recv callback runs in WiFi task context on core 0.** Copy into a volatile struct with a
  timestamp and return. Never touch ESCs, never block. Arduino runs `loop()` on core 1 and the WiFi
  stack on core 0, so the separation is already correct by default.
- **Drop stale sequence numbers.** ESP-NOW retries can deliver out of order. Reject any `seq` not
  strictly greater than the last accepted one.
- **Asymmetric hysteresis.** Leave the ESP-NOW source immediately on staleness. Require
  `FRESH_STREAK_REQUIRED` consecutive good packets to re-enter it. Symmetric thresholds flap at the
  edge of range and the robot oscillates between sources.

### New: linear slew limiter on the arbitrated output

The rate comes straight from the existing Jetson config defaults
(`include/transmitter/config.hpp:62,68`): `max_motor_rpm_per_sec / max_motor_rpm`, so
`3247.0 / 1500.0 = 2.165` per second in normalized units, or **216.5 percent per second**.

```c
// Slew-limit the arbitrated command so a source switch, an ESP-NOW dropout, or an orientation
// flip cannot step the wheels past the slip threshold. Applying the same rate the Jetson already
// applies is idempotent: when the trainer path is driving, this is a no-op.
const float MAX_A_PERCENT_PER_SEC = 216.5f;

float limit_a_percent(float requested, float dt) {
    static float prev = 0.0f;
    const float max_delta = MAX_A_PERCENT_PER_SEC * dt;
    prev = constrain(requested, prev - max_delta, prev + max_delta);
    return prev;
}
```

Reset `prev` to zero on every `stop_escs()` path so re-arming always ramps from standstill.

### Edit: `firmware/mr_stabs_mk2/src/main.cpp`

Insert the substitution after the armed gate at line 323:

```c
// ESP-NOW command substitution. Runs only after every CRSF gate has passed, so ESP-NOW can
// never arm the robot or extend its armed window. flip_switch_state UP is the pilot's
// "accept autonomous commands" position; anything else forces the trainer channel values.
bool espnow_active = false;
if (radio_data->flip_switch_state == crsf_bridge::UP && espnow.fresh(now_ms)) {
    const auto &cmd = espnow.latest();
    radio_data->a_percent = cmd.a_percent;
    radio_data->b_percent = cmd.b_percent;
    espnow_active = true;
}
```

Then apply the limiter after the inversion at line 377, before `mix_motor_outputs` at line 387.
Gate it on `flip_switch_state == UP` so the pilot keeps unlimited stick authority in manual mode,
matching how the Jetson-side limiter gates on the trainer enable channel today:

```c
     if (is_upside_down) radio_data->a_percent *= -1;
+    if (radio_data->flip_switch_state == crsf_bridge::UP) {
+        radio_data->a_percent = limit_a_percent(radio_data->a_percent, dt);
+    }
```

Mode-gate the radio setup. Combat mode drops the AP entirely so its periodic management frames and
the dashboard's TX bursts stop adding contention and jitter to the command path:

```
tuning -> combat:  diag_server.end() -> WiFi.softAPdisconnect(true) -> WiFi.mode(WIFI_STA)
                   -> sweep_for_dongle()   // sets the channel, see channel selection above
                   -> espnow.begin(...) -> esp_now_set_peer_rate_config(PEER_MAC, 6 Mbps OFDM)

combat -> tuning:  espnow.end() -> WiFi.softAP(ssid, pass, resolved_channel)
                   -> setup_ota() -> diag_server.begin(tunables)
```

No `WIFI_PROTOCOL_LR` in either direction, per the standing decision. That also simplifies the
transition, since there is no protocol mode to clear before the AP comes up.

Peers need re-registering after any mode or channel change, so treat the transition as an explicit
sequence rather than assuming the peer survives. The transition costs 100-300 ms, which is free
because the mode toggle already stops the motors.

Replace the hardcoded channel at `main.cpp:224` with a resolved channel used by both the SoftAP and
ESP-NOW. The firmware does not choose it. It is whatever the sweep locked onto, or the override:

```c
// 0 selects the sweep. Set a real channel to pin both ends, which is what bench runs should do so
// results are reproducible against a known channel.
const uint8_t ESPNOW_CHANNEL_OVERRIDE = 0;

// Sweep candidates in order. Last-known-good is tried ahead of these at runtime.
const uint8_t CHANNEL_CANDIDATES[] = {1, 6, 11};
const uint32_t CHANNEL_DWELL_MS = 25;   // ~6 dongle repeats at 250 Hz, plus switch settling

// Well above COMMAND_STALE_MS (60). A marginal link should fall back to the trainer path and stay
// there, not drop into a sweep that loses the packets which would have recovered it.
const uint32_t REACQUIRE_SILENCE_MS = 500;
```

Two consequences for the mode transition above. The tuning-mode SoftAP comes up on the resolved
channel rather than a fixed one, so a laptop reconnecting to the same SSID may land on a different
channel between boots. And combat mode has no AP, so the sweep is the only re-acquisition path
during a match. It has to work without one.

### Edit: `firmware/mr_stabs_mk2/include/diagnostics_server.h`

`wifi_clients` becomes meaningless once the AP is gone. Replace it and add the fields the Jetson
needs to evaluate the link:

```c
-    uint8_t wifi_clients;
+    uint8_t  espnow_lq;        // ACK success percent over a sliding window
+    bool     espnow_active;    // ESP-NOW supplied this tick's a/b percent
+    uint32_t espnow_age_ms;    // time since last accepted ESP-NOW command
+    uint32_t espnow_seq;       // last accepted sequence number
+    uint32_t jetson_tx_us;     // echoed for round-trip measurement
+    uint8_t  espnow_channel;   // channel the sweep locked onto, 0 while sweeping
```

`espnow_channel` is what makes a bad auto pick diagnosable after the fact. Without it in the MCAP,
a match with poor LQ has no attributable cause.

Four call sites reference `wifi_clients`: `main.cpp:276`, `:312`, `:344`, `:409`.

### Delete (Phase 3): `DiagnosticsServer`

Remove `diagnostics_server.cpp/.h`, and drop `me-no-dev/ESPAsyncWebServer` and `me-no-dev/AsyncTCP`
from `firmware/mr_stabs_mk2/platformio.ini`. AsyncTCP runs its own task and has a long history of
stack-overflow crashes under load. A firmware crash mid-match costs the match.

`ArduinoOTA` stays, gated to tuning mode. OTA over ESP-NOW is buildable (`Update.h` is
transport-agnostic, and 866 KB at 240 usable bytes per packet is about 3,600 packets, so 10-15
seconds stop-and-wait) but there is no reason to write it while the pit AP works.

## Jetson changes

`OpenTxTransmitter` and its header are untouched.

### New: `include/transmitter/espnow_transmitter.hpp`

```cpp
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "serial/serial_port.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/config.hpp"
#include "transmitter/differential_drive_processor.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

#pragma pack(push, 1)
struct EspNowWireHeader {
    uint16_t magic;
    uint8_t type;
    uint8_t length;
};

struct EspNowCommandPayload {
    uint32_t seq;
    uint32_t jetson_tx_us;
    float linear;
    float angular;
};

struct EspNowLinkStatsPayload {
    uint32_t sent;
    uint32_t acked;
    uint32_t failed;
    uint32_t last_rtt_us;
};
#pragma pack(pop)

/**
 * Sends autonomous velocity commands to mr_stabs_mk2 over an ESP-NOW USB dongle, cutting the
 * handset mixer and the Crossfire frame period out of the command path.
 *
 * Send-only by design. This transmitter has no handset channel input, so it reports no command
 * feedback and no init button press; those stay with OpenTxTransmitter. It also applies no linear
 * slew limit: the limiter depends on the trainer enable channel, which is not observable here, so
 * rate limiting happens in firmware on the arbitrated output instead.
 *
 * ESP-NOW carries no arm authority. The firmware substitutes these linear/angular values only
 * after every CRSF gate has passed, and falls back to the trainer channels when they go stale.
 *
 * Usable standalone for bench work without a handset, or under CompositeTransmitter alongside
 * OpenTxTransmitter for normal operation.
 */
class EspNowTransmitter : public TransmitterInterface {
   public:
    EspNowTransmitter(const EspNowTransmitterConfiguration &config,
                      std::shared_ptr<ClockInterface> clock);

    bool initialize() override;
    /** Drains dongle telemetry and link stats. Always returns an empty feedback map. */
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    /** No handset channels on this transport. Always false. */
    bool did_init_button_press() override { return false; }
    bool is_connected() const override { return dongle_.is_open(); }
    void enable() override { enabled_ = true; }
    void disable() override { enabled_ = false; }

   private:
    EspNowTransmitterConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;
    DifferentialDriveProcessor processor_;

    SerialPort dongle_;
    std::vector<uint8_t> rx_buffer_;
    uint32_t seq_ = 0;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point next_reconnect_attempt_ =
        std::chrono::steady_clock::now();

    bool reconnect_if_needed();
    void drain_dongle();
    void handle_frame(uint8_t type, const uint8_t *payload, uint8_t length);
    bool write_frame(uint8_t type, const void *payload, uint8_t length);
    static uint16_t crc16_ccitt(const uint8_t *data, size_t length);
};

}  // namespace auto_battlebot
```

### New: `src/transmitter/espnow_transmitter.cpp`

```cpp
#include "transmitter/espnow_transmitter.hpp"

#include <cstddef>
#include <cstring>

namespace auto_battlebot {
namespace {
constexpr auto kReconnectInterval = std::chrono::seconds(1);
constexpr uint16_t kWireMagic = 0xB07B;
constexpr uint8_t kMsgCommand = 0x01;
constexpr uint8_t kMsgTelemetry = 0x02;
constexpr uint8_t kMsgLinkStats = 0x04;
constexpr size_t kMaxFrameSize = 512;

uint32_t monotonic_us() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}
}  // namespace

EspNowTransmitter::EspNowTransmitter(const EspNowTransmitterConfiguration &config,
                                     std::shared_ptr<ClockInterface> clock)
    : config_(config),
      logger_(DiagnosticsLogger::get_logger("espnow_transmitter")),
      clock_(std::move(clock)),
      processor_(
          {
              .velocity_saturation_limit = config.velocity_saturation_limit,
              .zero_deadzone_percent = config.zero_deadzone_percent,
              .lifted_deadzone_percent = config.lifted_deadzone_percent,
              .reverse_linear = config.reverse_linear_channel,
              .reverse_angular = config.reverse_angular_channel,
          },
          logger_) {}

bool EspNowTransmitter::initialize() { return reconnect_if_needed(); }

CommandFeedback EspNowTransmitter::update() {
    drain_dongle();
    // Feedback comes from handset channels, which this transport does not carry. Returning empty
    // keeps CompositeTransmitter's merge unambiguous.
    return {};
}

void EspNowTransmitter::send(VelocityCommand command) {
    reconnect_if_needed();
    if (!dongle_.is_open() || !enabled_) return;

    // No slew limit here. The firmware rate-limits the arbitrated output, which is the only place
    // that sees which source actually won.
    const auto processed = processor_.process(command);

    const EspNowCommandPayload payload{
        .seq = ++seq_,
        .jetson_tx_us = monotonic_us(),
        .linear = static_cast<float>(processed.linear),
        .angular = static_cast<float>(processed.angular),
    };

    if (!write_frame(kMsgCommand, &payload, sizeof(payload))) {
        logger_->warning("dongle_write_failed", {{"seq", static_cast<int>(payload.seq)}});
        dongle_.close();
        next_reconnect_attempt_ = std::chrono::steady_clock::now();
        return;
    }

    logger_->debug("espnow_send", {{"seq", static_cast<int>(payload.seq)},
                                   {"linear", processed.linear},
                                   {"angular", processed.angular}});
}

bool EspNowTransmitter::write_frame(uint8_t type, const void *payload, uint8_t length) {
    std::vector<uint8_t> frame(sizeof(EspNowWireHeader) + length + sizeof(uint16_t));
    const EspNowWireHeader header{kWireMagic, type, length};
    std::memcpy(frame.data(), &header, sizeof(header));
    std::memcpy(frame.data() + sizeof(header), payload, length);

    const size_t crc_offset = offsetof(EspNowWireHeader, type);
    const uint16_t crc = crc16_ccitt(frame.data() + crc_offset, 2 + length);
    std::memcpy(frame.data() + sizeof(header) + length, &crc, sizeof(crc));
    return dongle_.write(frame.data(), frame.size());
}

void EspNowTransmitter::drain_dongle() {
    if (!dongle_.is_open()) return;

    auto bytes = dongle_.read_available();
    if (bytes.empty()) return;
    rx_buffer_.insert(rx_buffer_.end(), bytes.begin(), bytes.end());

    // Scan for magic, consume complete frames, discard leading garbage.
    size_t consumed = 0;
    while (rx_buffer_.size() - consumed >= sizeof(EspNowWireHeader)) {
        const uint8_t *base = rx_buffer_.data() + consumed;
        uint16_t magic;
        std::memcpy(&magic, base, sizeof(magic));
        if (magic != kWireMagic) {
            consumed++;
            continue;
        }

        const uint8_t length = base[3];
        const size_t frame_size = sizeof(EspNowWireHeader) + length + sizeof(uint16_t);
        if (rx_buffer_.size() - consumed < frame_size) break;  // wait for the rest

        uint16_t crc;
        std::memcpy(&crc, base + sizeof(EspNowWireHeader) + length, sizeof(crc));
        if (crc == crc16_ccitt(base + offsetof(EspNowWireHeader, type), 2 + length)) {
            handle_frame(base[2], base + sizeof(EspNowWireHeader), length);
        } else {
            logger_->debug("crc_mismatch", {{"type", static_cast<int>(base[2])}});
        }
        consumed += frame_size;
    }

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + consumed);
    if (rx_buffer_.size() > kMaxFrameSize) rx_buffer_.clear();  // desynced, resynchronize
}

void EspNowTransmitter::handle_frame(uint8_t type, const uint8_t *payload, uint8_t length) {
    switch (type) {
        case kMsgLinkStats: {
            if (length != sizeof(EspNowLinkStatsPayload)) return;
            EspNowLinkStatsPayload stats;
            std::memcpy(&stats, payload, sizeof(stats));
            const double lq =
                stats.sent > 0 ? 100.0 * stats.acked / static_cast<double>(stats.sent) : 0.0;
            logger_->debug("link_stats", {{"sent", static_cast<int>(stats.sent)},
                                          {"acked", static_cast<int>(stats.acked)},
                                          {"failed", static_cast<int>(stats.failed)},
                                          {"lq_percent", lq},
                                          {"last_rtt_us", static_cast<int>(stats.last_rtt_us)}});
            break;
        }
        case kMsgTelemetry:
            // Phase 1 logs the raw struct. Phase 3 forwards it to the diagnostics dashboard.
            logger_->debug("telemetry", {{"bytes", static_cast<int>(length)}});
            break;
        default:
            break;
    }
}

bool EspNowTransmitter::reconnect_if_needed() {
    if (dongle_.is_open()) return true;

    const auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_attempt_) return false;
    next_reconnect_attempt_ = now + kReconnectInterval;

    if (!dongle_.open(config_.dongle_serial_path, config_.dongle_baud_rate)) {
        logger_->debug("dongle_not_found", {{"path", config_.dongle_serial_path}});
        return false;
    }
    logger_->info("dongle_opened", {{"path", config_.dongle_serial_path}});
    rx_buffer_.clear();
    return true;
}

uint16_t EspNowTransmitter::crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace auto_battlebot
```

### New: `include/transmitter/composite_transmitter.hpp`

```cpp
#pragma once

#include <memory>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

/**
 * Drives several transmitters from one command stream.
 *
 * send(), enable(), and disable() fan out to every child. The first child in config order is
 * primary and owns everything read-side: connection state, init button edges, and command
 * feedback on key collision. Non-primary children are accelerators, so their initialize()
 * failures are logged and ignored.
 *
 * The composite does no command preprocessing. Children are responsible for their own scaling,
 * and any rate limiting needed to make two transports agree belongs downstream of the arbitration
 * that picks between them.
 */
class CompositeTransmitter : public TransmitterInterface {
   public:
    explicit CompositeTransmitter(std::vector<std::shared_ptr<TransmitterInterface>> children);

    bool initialize() override;
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    bool is_connected() const override { return children_.front()->is_connected(); }
    void enable() override;
    void disable() override;

   private:
    std::vector<std::shared_ptr<TransmitterInterface>> children_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
};

}  // namespace auto_battlebot
```

### New: `src/transmitter/composite_transmitter.cpp`

```cpp
#include "transmitter/composite_transmitter.hpp"

#include <stdexcept>

namespace auto_battlebot {

CompositeTransmitter::CompositeTransmitter(
    std::vector<std::shared_ptr<TransmitterInterface>> children)
    : children_(std::move(children)),
      logger_(DiagnosticsLogger::get_logger("composite_transmitter")) {
    if (children_.empty()) {
        throw std::invalid_argument("CompositeTransmitter requires at least one child");
    }
}

bool CompositeTransmitter::initialize() {
    bool primary_ok = false;
    for (size_t i = 0; i < children_.size(); i++) {
        const bool ok = children_[i]->initialize();
        if (i == 0) {
            primary_ok = ok;
        } else if (!ok) {
            // Non-primary children are accelerators. A failure costs latency, not control.
            logger_->warning("child_initialize_failed", {{"index", static_cast<int>(i)}});
        }
    }
    return primary_ok;
}

CommandFeedback CompositeTransmitter::update() {
    CommandFeedback merged;
    // Reverse order so the primary is applied last and wins on key collision. Every child is
    // still polled, which is what drains their receive paths.
    for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        for (const auto &[frame_id, command] : (*it)->update().stick_commands) {
            merged.stick_commands[frame_id] = command;
        }
    }
    return merged;
}

void CompositeTransmitter::send(VelocityCommand command) {
    for (const auto &child : children_) child->send(command);
}

bool CompositeTransmitter::did_init_button_press() {
    // No short-circuit: every child must be polled so its edge latch clears.
    bool pressed = false;
    for (const auto &child : children_) {
        if (child->did_init_button_press()) pressed = true;
    }
    return pressed;
}

void CompositeTransmitter::enable() {
    for (const auto &child : children_) child->enable();
}

void CompositeTransmitter::disable() {
    for (const auto &child : children_) child->disable();
}

}  // namespace auto_battlebot
```

### Edit: `include/transmitter/config.hpp`

`EspNowTransmitterConfiguration` carries only what it uses. It has no drivetrain geometry fields
(no feedback to scale) and no `max_motor_rpm_per_sec` (no slew limiter).

```cpp
struct EspNowTransmitterConfiguration : public TransmitterConfiguration {
    /** Serial device for the ESP-NOW USB dongle. Use a udev symlink, not a raw ttyACM path:
     *  the handset also enumerates as ttyACM and enumeration order is not stable. */
    std::string dongle_serial_path = "/dev/espnow_dongle";
    int dongle_baud_rate = 921600;

    /** These five must match the OpenTxTransmitter section exactly, or the two transports will
     *  disagree by a fixed offset that the firmware slew limiter cannot correct. */
    double velocity_saturation_limit = 1.0;
    double zero_deadzone_percent = 0.0;
    double lifted_deadzone_percent = 0.0;
    bool reverse_linear_channel = false;
    bool reverse_angular_channel = false;

    EspNowTransmitterConfiguration() { type = "EspNowTransmitter"; }

    // No PARSE_FIELD_INT macro exists (config_factory.hpp:102-118 defines STRING, DOUBLE, and
    // BOOL only), so int fields need an explicit override. Same reason
    // OpenTxTransmitterConfiguration hand-writes its parse_fields at config.hpp:77.
    void parse_fields(ConfigParser &parser) override {
        dongle_serial_path = parser.get_optional_string("dongle_serial_path", dongle_serial_path);
        dongle_baud_rate =
            static_cast<int>(parser.get_optional_int("dongle_baud_rate", dongle_baud_rate));
        velocity_saturation_limit =
            parser.get_optional_double("velocity_saturation_limit", velocity_saturation_limit);
        zero_deadzone_percent =
            parser.get_optional_double("zero_deadzone_percent", zero_deadzone_percent);
        lifted_deadzone_percent =
            parser.get_optional_double("lifted_deadzone_percent", lifted_deadzone_percent);
        reverse_linear_channel =
            parser.get_optional_bool("reverse_linear_channel", reverse_linear_channel);
        reverse_angular_channel =
            parser.get_optional_bool("reverse_angular_channel", reverse_angular_channel);
    }
};

struct CompositeTransmitterConfiguration : public TransmitterConfiguration {
    /** Child transmitters in priority order. The first is primary and owns all read-side
     *  behavior. Defined as an array of tables so ordering is guaranteed, matching the
     *  LabelToKeypointMapConfiguration pattern in config/enum_map_config.hpp. */
    std::vector<std::unique_ptr<TransmitterConfiguration>> children;

    CompositeTransmitterConfiguration() { type = "CompositeTransmitter"; }

    /** Defined in config.cpp: recurses through parse_transmitter_config. */
    void parse_fields(ConfigParser &parser) override;
};
```

### Edit: `src/transmitter/config.cpp`

```cpp
REGISTER_CONFIG(TransmitterConfiguration, EspNowTransmitterConfiguration, "EspNowTransmitter")
REGISTER_CONFIG(TransmitterConfiguration, CompositeTransmitterConfiguration, "CompositeTransmitter")

void CompositeTransmitterConfiguration::parse_fields(ConfigParser &parser) {
    const toml::array *array_ptr = parser.get_array("children");
    if (!array_ptr) {
        throw ConfigValidationError(
            "Missing required field 'children' in [transmitter] (expected array of tables)");
    }
    for (const auto &item : *array_ptr) {
        const toml::table *entry = item.as_table();
        if (!entry) {
            throw ConfigValidationError("Each entry in 'children' must be a table");
        }
        ConfigParser child_parser(*entry, "transmitter.children");
        auto child = parse_transmitter_config(child_parser);
        if (child->type == "CompositeTransmitter") {
            throw ConfigValidationError("CompositeTransmitter cannot nest inside itself");
        }
        children.push_back(std::move(child));
    }
    if (children.empty()) {
        throw ConfigValidationError("'children' must contain at least one transmitter");
    }
}
```

And in `make_transmitter` (`src/transmitter/config.cpp:36`):

```cpp
} else if (config.type == "EspNowTransmitter") {
    return std::make_shared<EspNowTransmitter>(
        config_cast<EspNowTransmitterConfiguration>(config), std::move(clock));
} else if (config.type == "CompositeTransmitter") {
    const auto &composite = config_cast<CompositeTransmitterConfiguration>(config);
    std::vector<std::shared_ptr<TransmitterInterface>> children;
    children.reserve(composite.children.size());
    for (const auto &child_config : composite.children) {
        children.push_back(make_transmitter(*child_config, clock));  // copy, do not move: reused
    }
    return std::make_shared<CompositeTransmitter>(std::move(children));
}
```

`CompositeTransmitterConfiguration` holds `unique_ptr`s and is therefore move-only. Nothing in the
current config path copies a `TransmitterConfiguration`, so this is fine, but a future
`config_cast` by value would break it.

### Config

```toml
[transmitter]
type = "CompositeTransmitter"

[[transmitter.children]]
type = "OpenTxTransmitter"
linear_channel = 0
angular_channel = 1
trainer_enable_channel = 7
velocity_saturation_limit = 1.0
zero_deadzone_percent = 0.0
lifted_deadzone_percent = 0.0
reverse_linear_channel = false
reverse_angular_channel = false
max_motor_rpm = 1500.0
max_motor_rpm_per_sec = 3247.0

[[transmitter.children]]
type = "EspNowTransmitter"
dongle_serial_path = "/dev/espnow_dongle"
# These five must match the OpenTxTransmitter child above.
velocity_saturation_limit = 1.0
zero_deadzone_percent = 0.0
lifted_deadzone_percent = 0.0
reverse_linear_channel = false
reverse_angular_channel = false
```

Because `config/` is an extends chain, put the shared five in the base file and let both children
inherit rather than repeating them per platform.

### udev rule

The handset (VID 0x0483, PID 0x5740) and the dongle both enumerate as `ttyACM`, and enumeration
order is not stable across boots. `find_opentx_device()` already handles the handset by VID/PID;
give the dongle a stable symlink. `/etc/udev/rules.d/99-espnow-dongle.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", SYMLINK+="espnow_dongle"
```

Confirm the product ID with `lsusb` after flashing, since it varies by Arduino core USB mode.

## Phases

### Phase 1: shadow mode, measure before committing

No control change. The firmware receives ESP-NOW commands, logs them, and keeps driving from the
trainer channels. `espnow_active` is recorded but never acted on.

Build only:

- `espnow_link` receive path plus telemetry send
- `EspNowTransmitter` and `CompositeTransmitter`, with the firmware substitution disabled
- Extra `diag_data_t` fields

Measurements to collect, in the arena and not on the bench:

1. **Round trip.** `jetson_tx_us` echoed in telemetry. One-way is approximately half.
2. **Trainer path delay.** Cross-correlate the Jetson's commanded linear against `a_percent` as
   reported in telemetry. The lag at peak correlation is the trainer path delay. The difference
   between this and (1) is the actual latency saving, which either justifies the work or does not.
3. **Loss rate.** Sequence gaps at the firmware plus ACK failures from `0x04` link stats.
4. **Dropout distribution.** Gap durations, not just a mean. The p90 and max matter far more, the
   same way they did for the perception dropout baseline.
5. **Path agreement.** Log `a_percent` from both sources on the same tick. Any steady-state offset
   means the two `DifferentialDriveProcessor` configs have drifted apart.
6. **ACK rate at 6 Mbps.** The standing decision gives up ~10 dB by refusing LR, so this is the
   number that says whether that was affordable in the arena. Record it per match position, not
   pooled: range failures are positional.
7. **Source switch rate.** How often arbitration moves between ESP-NOW and the trainer path, given
   `FRESH_STREAK_REQUIRED = 3`. Confirms the faster re-entry is not causing churn.

The Tradeoffs section predicts ~5.5 ms at 6 Mbps. Measurement 2 either confirms that or points at
the 33.3 ms command interval as the better target.

### Phase 2: promote to command source

Enable the substitution and the firmware slew limiter. Verify in this order:

1. On the bench, ESP-NOW commanding full forward, flip SF off. Motors must stop. This single test
   validates the entire authority split.
2. Flip `flip_switch_state` out of UP mid-command. Output must fall back to trainer channels and
   ramp rather than step.
3. Pull dongle power mid-command. Output must fall back within `COMMAND_STALE_MS`.
4. Confirm the firmware limiter is a no-op when the trainer path is driving, by comparing logged
   `a_percent` against the Jetson's commanded value.
5. Only then, in the box.

### Phase 3: delete the on-robot dashboard

Move telemetry rendering to the Jetson web tool from `004342f`. Delete `DiagnosticsServer`, drop
`ESPAsyncWebServer` and `AsyncTCP` from `platformio.ini`, and record the new `firmware.bin` size and
free heap against the 866 KB baseline. That number sets the headroom available for the camera port.

## Risks

**The ~5 ms saving is computed, not measured.** Every checkable radio setting already came back
optimal, so there is no cheaper alternative left to try, but the figure still rests on frame sizes
and 802.11 timing constants. Phase 1 confirms it. If it measures materially lower, the fix is to
attack the 33.3 ms command interval via `control_loop_thread_plan.md` instead, not to abandon the
transport work.

**Dropping LR costs range.** The standing decision takes 6 Mbps OFDM for ~2.7 ms, giving up roughly
10 dB of link budget. In a steel-framed arena with the Jetson outside, that is the one accepted
cost in this plan with real failure potential. Phase 1 measures it as ACK rate, and the mitigation
is antenna and placement work rather than reverting to LR.

**NHRL rules.** Moving primary command onto a custom link is a stricter rules question than
telemetry. Confirm with the event before building Phase 2. This can invalidate the plan, so ask
first.

**Command repeat rate.** Without the dongle-side 250 Hz repeat, ESP-NOW's dropout tail is worse
than Crossfire's despite its better median. This is a correctness requirement, not an
optimization.

**Faster source switching.** `FRESH_STREAK_REQUIRED = 3` re-enters the fast path 8 ms sooner than 5
would, at the cost of more switching at the edge of range. The firmware slew limiter absorbs the
resulting steps, but Phase 1 measurement 4 should confirm the switch rate stays sane before
Phase 2.

**Duplicated `DifferentialDriveProcessor` config.** The two children each hold their own copy of the
five processor fields. If they drift, the transports disagree by a fixed offset, and the firmware
slew limiter cannot correct an offset (it limits rate, not value). Phase 1 measurement 5 catches
this. A construction-time equality check across children would catch it earlier and is worth adding
if the config proves easy to get wrong.

**No frequency hopping.** ESP-NOW stays on one channel for the run. A channel that goes bad
mid-match takes the link out with nothing to hop to, and the sweep will not help because the dongle
is still transmitting there. Mitigation is that both paths carry every command: losing ESP-NOW
costs latency, not control.

**Auto channel selection.** The scan can rank a channel quiet that is loud with traffic it cannot
see, and the sweep is a new failure surface on a path that used to be a flashed constant. Worst
case is a robot that never locks and runs the whole match on the trainer path, which is exactly
today's behavior. Test the sweep with the dongle deliberately parked on each of 1, 6, and 11, and
with the dongle off entirely, and confirm the robot settles into trainer-only rather than sweeping
forever and stalling `loop()`. Full reasoning in the channel selection section.

**Mode transition.** Tearing WiFi down and back up is the most likely thing to break, since peers
need re-registering and the peer rate config must be reapplied after any channel change. The sweep
changes channels at runtime, so it hits this path too, not only the mode toggle. Test combat to
tuning to combat repeatedly, and confirm OTA still works after a round trip.

**USB3 noise.** The ZED 2i emits broadband noise near 2.4 GHz. Keep the dongle on a short extension
away from the camera cable and ferrite the ZED cable. This looks like arena interference and is
easy to misattribute.

**ESP-NOW v2.** The 1490-byte payload would cut OTA packet count sixfold, but it needs IDF 5.4.
Check what the pinned `espressif32` platform actually provides before designing around it.

## Next steps

1. Raise the model's CRSF baud to 921600 in Model Setup. Roughly 1 ms off the trainer path, which
   is also the ESP-NOW fallback path, for the cost of one menu change. Free latency, no code.
2. Land [`control_loop_thread_plan.md`](control_loop_thread_plan.md) before this plan's command
   path. It is worth ~15 ms against this plan's ~5 ms, needs no new hardware, and it changes the
   requirements here (send rate, per-child transmit caps).
3. Confirm the NHRL rules question. Only gates the command path.
4. Build the dongle firmware. Confirm `esp_now_add_peer` succeeds with `ifidx = WIFI_IF_AP` (the
   most common silent failure), set the 6 Mbps rate config, and implement forward-on-arrival plus
   the 250 Hz repeat. Pin the channel with `ESPNOW_CHANNEL_OVERRIDE` for this step. Bring up the
   boot scan and the robot sweep only after a pinned link measures clean, so a sweep bug and a
   radio bug cannot be confused for each other.
5. Land `CompositeTransmitter` first, with `OpenTxTransmitter` as its only child. It should be a
   behavioral no-op against the current config, which makes it cheap to verify and de-risks the
   fan-out before any radio work.
6. Add the `diag_data_t` field swap. Independent of everything else.
7. Run Phase 1 and get the latency, loss, ACK-rate, and path-agreement numbers before writing any
   of Phase 2.
