"""BLHeli_32 settings record: the 192-byte payload of the 0xF800 page, firmware 32.x layout 48.

Offsets follow BLHeliSuite32xl 1.0.4.4's `Eep_*` record order, checked against two decrypted
Flycolor Raptor 5 pages (firmware/blheli/captures/). Encodings come from stepping every
BLHeliSuite slider and reading back its model byte and label (firmware/blheli/README.md).
`verified=False` marks fields whose offset or encoding has not been confirmed on hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field

LAYOUT_REVISION = 48
FW_MAIN = 32


@dataclass(frozen=True)
class Field:
    key: str
    offset: int
    width: int = 1  # bytes; 2 = little-endian u16
    kind: str = "int"  # int | enum | text | readonly
    lo: int = 0
    hi: int = 255
    options: dict[int, str] = field(default_factory=dict)
    unit: str = ""
    off_value: int | None = None  # raw value BLHeliSuite labels "Off"
    verified: bool = True
    doc: str = ""

    def read(self, payload: bytes | bytearray) -> int:
        return int.from_bytes(payload[self.offset : self.offset + self.width], "little")

    def label(self, raw: int) -> str:
        if raw in self.options:
            return self.options[raw]
        if self.off_value is not None and raw == self.off_value:
            return "Off"
        return f"{raw}{(' ' + self.unit) if self.unit else ''}"


def _range(lo: int, hi: int, fmt: str) -> dict[int, str]:
    return {v: fmt.format(v) for v in range(lo, hi + 1)}


DIRECTION = {
    1: "Normal",
    2: "Reversed",
    3: "Bidirectional 3D",
    4: "Bidirectional 3D Rev.",
    5: "Bidirectional Soft",
    6: "Bidirectional Soft Rev.",
}
ON_OFF = {0: "Off", 1: "On"}

FIELDS: list[Field] = [
    Field("fw_main_revision", 0, kind="readonly", doc="Firmware major (32)"),
    Field("fw_sub_revision", 1, kind="readonly", doc="Firmware minor (90 = 32.9)"),
    Field("layout_revision", 2, kind="readonly", doc="Settings layout revision (48)"),
    Field(
        "direction",
        3,
        kind="enum",
        lo=1,
        hi=6,
        options=DIRECTION,
        doc="Motor direction. Bidirectional modes put zero at center throttle",
    ),
    Field(
        "rampup_power",
        4,
        lo=3,
        hi=150,
        unit="%",
        doc="Max power at low rpm and during startup; also limits power on direction reversal",
    ),
    Field(
        "pwm_frequency_low",
        5,
        lo=16,
        hi=128,
        unit="kHz",
        doc="Motor PWM frequency at low throttle (confirmed by a Write Setup capture)",
    ),
    Field(
        "comm_timing",
        6,
        kind="enum",
        lo=0,
        hi=31,
        options={0: "Auto", **_range(1, 31, "{} deg")},
        doc="Commutation timing in 15/16 deg steps; 0 = auto",
    ),
    Field(
        "demag_compensation",
        7,
        kind="enum",
        lo=1,
        hi=4,
        options={1: "Off", 2: "Low", 3: "High", 4: "Very High"},
        doc="Cuts power before commutation when winding demag is long",
    ),
    Field(
        "min_throttle",
        8,
        width=2,
        lo=900,
        hi=2100,
        unit="us",
        doc="PWM input min. No effect with DShot",
    ),
    Field(
        "center_throttle",
        10,
        width=2,
        lo=900,
        hi=2100,
        unit="us",
        doc="PWM input center, bidirectional only. No effect with DShot",
    ),
    Field(
        "max_throttle",
        12,
        width=2,
        lo=900,
        hi=2100,
        unit="us",
        doc="PWM input max. No effect with DShot",
    ),
    Field(
        "throttle_cal_enable",
        14,
        kind="enum",
        lo=0,
        hi=1,
        options=ON_OFF,
        doc="Allow stick calibration at power-up",
    ),
    Field(
        "temperature_protection",
        15,
        lo=0,
        hi=255,
        unit="C",
        off_value=0,
        doc="MCU temperature where power limiting starts; 0 = off",
    ),
    Field(
        "low_voltage_protection",
        16,
        kind="enum",
        lo=0,
        hi=16,
        options={0: "Off", **{n: f"{2.4 + 0.1 * n:.2f} V" for n in range(1, 17)}},
        doc="Per-cell cutoff 2.5-4.0 V",
    ),
    Field(
        "current_protection",
        17,
        lo=0,
        hi=255,
        unit="A",
        off_value=0,
        verified=False,
        doc="Per-ESC current limit. Reads 0xFF on ESCs without current sense",
    ),
    Field(
        "low_rpm_power_protect",
        18,
        kind="enum",
        lo=0,
        hi=2,
        options={0: "Off", 1: "On", 2: "On Adaptive"},
        doc="Power limit at low rpm; adaptive scales with kV x voltage",
    ),
    Field(
        "brake_on_stop", 19, lo=0, hi=100, unit="%", off_value=0, doc="Brake force at zero throttle"
    ),
    Field("startup_beep_volume", 20, lo=0, hi=254, off_value=0),
    Field(
        "beacon_volume",
        21,
        lo=0,
        hi=255,
        off_value=0,
        doc="Beacon and DShot tone volume. High values heat the motor",
    ),
    Field(
        "beacon_delay",
        22,
        width=2,
        lo=0,
        hi=3600,
        unit="s",
        off_value=0,
        doc="Zero-throttle time before beacon beeps",
    ),
    Field(
        "led_control",
        24,
        kind="enum",
        lo=0,
        hi=3,
        options={0: "Off", 3: "On"},
        doc="LED on/off. 0 Off, 3 On on the one-LED Raptor 5; multi-LED bit layout unseen",
    ),
    Field(
        "max_acceleration",
        25,
        kind="enum",
        lo=0,
        hi=255,
        options={0: "Maximum", **{n: f"{n / 10:.1f} %/ms" for n in range(1, 256)}},
        doc="Power slew limit; 0 = unlimited",
    ),
    Field(
        "nondamped_mode",
        26,
        kind="enum",
        lo=0,
        hi=1,
        options=ON_OFF,
        doc="Turns off damped light (regen braking / active freewheel)",
    ),
    Field(
        "current_sense_cal",
        27,
        lo=0,
        hi=255,
        unit="%",
        verified=False,
        doc="Current sensor gain trim. Not shown for this ESC",
    ),
    Field(
        "music_note_config",
        28,
        lo=0,
        hi=255,
        doc="Startup music: high nibble = general note length, low nibble = interval (pause)",
    ),
    Field(
        "sine_mode",
        29,
        kind="enum",
        lo=0,
        hi=1,
        options=ON_OFF,
        doc="Sine power modulation. Disables variable PWM frequency",
    ),
    Field(
        "auto_telemetry",
        30,
        kind="enum",
        lo=0,
        hi=1,
        options=ON_OFF,
        doc="Autonomous telemetry every 32 ms",
    ),
    Field("stall_protection", 31, kind="enum", lo=0, hi=1, options={0: "Relaxed", 1: "Normal"}),
    Field(
        "sbus_channel", 32, kind="enum", lo=0, hi=17, options={**_range(0, 16, "CH {}"), 17: "Off"}
    ),
    Field("sport_physical_id", 33, lo=0, hi=28, off_value=0),
    Field(
        "pwm_frequency_high",
        34,
        kind="enum",
        lo=16,
        hi=129,
        options={**_range(16, 128, "{} kHz"), 129: "By RPM"},
        doc="Motor PWM frequency at high throttle; 129 = by RPM",
    ),
    Field("hw_voltage_sense_capable", 48, kind="readonly"),
    Field("hw_current_sense_capable", 49, kind="readonly", doc="255 = no current sensor"),
    Field("hw_led_capable_0", 50, kind="readonly"),
    Field("hw_led_capable_1", 51, kind="readonly"),
    Field("hw_led_capable_2", 52, kind="readonly"),
    Field("hw_led_capable_3", 53, kind="readonly"),
    Field("hw_pwm_freq_min", 54, kind="readonly", unit="kHz"),
    Field("hw_pwm_freq_max", 55, kind="readonly", unit="kHz"),
    Field("sport_capable", 62, kind="readonly"),
    Field("nondamped_capable", 63, kind="readonly"),
    Field("esc_layout", 64, width=32, kind="text", doc="'#Flycolor_Raptor_5#' padded"),
    Field("esc_mcu", 96, width=32, kind="text", doc="'#BLHeli_32*STM32G071x6#' padded"),
    Field("name", 128, width=16, kind="text", doc="User ESC name"),
    Field(
        "music_notes",
        144,
        width=48,
        kind="music",
        doc="Startup music, one byte per note, 0xFF ends it (see encode_music)",
    ),
]
BY_KEY = {f.key: f for f in FIELDS}
EDITABLE = [f for f in FIELDS if f.kind in ("int", "enum")]
# Payload bytes no field claims (Filler, FlashCounter, Filler2). Carried through unchanged.
RESERVED = sorted(set(range(192)) - {f.offset + i for f in FIELDS for i in range(f.width)})


class LayoutError(ValueError):
    pass


def check_layout(payload: bytes) -> None:
    fw, lay = payload[0], payload[2]
    if fw != FW_MAIN or lay != LAYOUT_REVISION:
        raise LayoutError(
            f"firmware {fw}.{payload[1]} layout {lay}: this table is for "
            f"{FW_MAIN}.x layout {LAYOUT_REVISION}"
        )


def decode(payload: bytes) -> dict[str, int | str]:
    check_layout(payload)
    out: dict[str, int | str] = {}
    for f in FIELDS:
        raw = payload[f.offset : f.offset + f.width]
        if f.kind == "text":
            out[f.key] = raw.decode("latin-1")
        elif f.kind == "music":
            out[f.key] = decode_music(payload)
        else:
            out[f.key] = f.read(payload)
    return out


def encode_name(payload: bytes, name: str) -> bytes:
    """Set the 16-character ESC name (printable ASCII, space padded like BLHeliSuite's)."""
    f = BY_KEY["name"]
    if len(name) > f.width or not all(32 <= ord(c) < 127 for c in name):
        raise LayoutError(f"name must be up to {f.width} printable ASCII characters")
    check_layout(payload)
    buf = bytearray(payload)
    buf[f.offset : f.offset + f.width] = name.ljust(f.width).encode("ascii")
    return bytes(buf)


def encode(payload: bytes, changes: dict[str, int]) -> bytes:
    """Apply raw-value changes to a payload read from the ESC. Everything else is kept."""
    check_layout(payload)
    buf = bytearray(payload)
    for key, value in changes.items():
        f = BY_KEY.get(key)
        if f is None:
            raise LayoutError(f"unknown field {key!r}")
        if f.kind not in ("int", "enum"):
            raise LayoutError(f"{key} is {f.kind}, not editable")
        if not f.lo <= value <= f.hi:
            raise LayoutError(f"{key}={value} outside {f.lo}..{f.hi}")
        if f.kind == "enum" and f.options and value not in f.options:
            raise LayoutError(f"{key}={value} is not one of {sorted(f.options)}")
        buf[f.offset : f.offset + f.width] = value.to_bytes(f.width, "little")
    values = {k: (BY_KEY[k].read(buf)) for k in ("min_throttle", "center_throttle", "max_throttle")}
    if not values["min_throttle"] < values["center_throttle"] < values["max_throttle"]:
        raise LayoutError(f"throttle needs min < center < max, got {values}")
    if BY_KEY["pwm_frequency_low"].read(buf) > BY_KEY["pwm_frequency_high"].read(buf):
        raise LayoutError("pwm_frequency_low must not exceed pwm_frequency_high")
    return bytes(buf)


def parse_value(key: str, text: str) -> int:
    """Raw value from a number or an option label ('High', '48 kHz', 'Off')."""
    f = BY_KEY[key]
    try:
        return int(text, 0)
    except ValueError:
        pass
    want = text.strip().lower()
    for raw, lab in f.options.items():
        if lab.lower() == want:
            return raw
    if f.off_value is not None and want == "off":
        return f.off_value
    raise LayoutError(f"{key}: {text!r} is not a number or one of the labels")


# Startup music. One byte per note: bits 7-6 length (0 = 1/8, 1 = 1/4, 2 = 1/2, 3 = 1/1),
# bits 5-0 = (octave - 4) * 16 + semitone. 0xFF ends the list. Decoded from BLHeliSuite's
# Write Setup for "C5 1/8 C#5 1/4 D6 1/2 C4 1/2 B5 1/1 A5 1/8" -> 10 51 A2 80 DB 19.
NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
_LENGTHS = {8: 0, 4: 1, 2: 2, 1: 3}
MUSIC = BY_KEY["music_notes"]


def decode_music(payload: bytes) -> str:
    """Note array -> BLHeliSuite script, e.g. 'A5 1/8 D5 1/4'."""
    notes = []
    for b in payload[MUSIC.offset : MUSIC.offset + MUSIC.width]:
        if b == 0xFF:
            break
        pitch, length = b & 0x3F, b >> 6
        octave, semitone = 4 + pitch // 16, pitch % 16
        name = NOTE_NAMES[semitone] if semitone < 12 else f"?{semitone}"
        denom = {v: k for k, v in _LENGTHS.items()}[length]
        notes.append(f"{name}{octave} 1/{denom}")
    return " ".join(notes)


def encode_music(payload: bytes, script: str) -> bytes:
    """Replace the note array with a script of 'NOTE OCTAVE 1/N' tokens (N in 1, 2, 4, 8)."""
    tokens = script.split()
    if len(tokens) % 2:
        raise LayoutError("music script needs NOTE+OCTAVE and 1/N pairs, e.g. 'A5 1/8'")
    out = bytearray()
    for note, dur in zip(tokens[0::2], tokens[1::2]):
        name, octave_text = note[:-1].upper(), note[-1]
        if name not in NOTE_NAMES or not octave_text.isdigit() or not 4 <= int(octave_text) <= 7:
            raise LayoutError(f"bad note {note!r}: use C..B with optional #, octave 4-7")
        if not dur.startswith("1/") or not dur[2:].isdigit() or int(dur[2:]) not in _LENGTHS:
            raise LayoutError(f"bad length {dur!r}: use 1/1, 1/2, 1/4 or 1/8")
        pitch = (int(octave_text) - 4) * 16 + NOTE_NAMES.index(name)
        out.append(_LENGTHS[int(dur[2:])] << 6 | pitch)
    if len(out) > MUSIC.width:
        raise LayoutError(f"music has {len(out)} notes, the ESC holds {MUSIC.width}")
    check_layout(payload)
    buf = bytearray(payload)
    buf[MUSIC.offset : MUSIC.offset + MUSIC.width] = bytes(out) + b"\xff" * (MUSIC.width - len(out))
    return bytes(buf)
