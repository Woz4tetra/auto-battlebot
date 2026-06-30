#!/usr/bin/env python3
"""otx_tool.py - read and edit OpenTX model files (Taranis X9D+ 2019, EEPROM v219).

Board note
----------
This targets the Taranis **X9D+ 2019** (firmware BOARD_NAME=x9d+2019), which has 9
switches SA..SI. The plain X9D has 8 (SA..SH) and the same 'otx3' fourcc, so the
file does not reveal the board; the firmware build does. The extra switch shifts
trainer/logical-switch source numbers (see MIXSRC_* below). Getting this wrong
silently points a mix at the wrong source, so the numbering here is board-locked.

Two file formats are supported
------------------------------
1. .otx  - a ZIP (Companion / desktop) containing:
     RADIO/radio.bin, RADIO/models.txt, MODELS/modelN.bin (uncompressed dumps)
2. .bin  - a single radio SD-card model file (MODELS/<name>.bin). On an EEPROM_RLC
     build these are RLE-compressed (8-byte header + compressed stream). The tool
     decompresses on load and recompresses on save (compressor is byte-exact with
     the radio's own output).

Each modelN.bin is an 8-byte header followed by a raw memory dump of the
firmware's `ModelData` struct (sizeof(g_model) bytes). There is no text and no
checksum: the bytes are the packed C struct exactly as it sits in radio RAM.

This tool decodes the two arrays we usually want to touch, the mixer table
(`mixData`) and the input/expo table (`expoData`), lets you edit them, and writes
the file back byte-for-byte except for the bytes you actually change.

How the layout was obtained (do not "eyeball" these numbers)
------------------------------------------------------------
The struct is full of board/version #ifdefs, so the offsets were taken from the
compiler, not by hand. For PCBX9D (LCD_W==212) at EEPROM_VER==219:

    sizeof(ModelHeader)=24  TimerData=16  MixData=20  LimitData=13  ExpoData=17
    offsetof(mixData)=76    offsetof(limitData)=1356  offsetof(expoData)=1772

Source numbering (sticks / inputs / trainer / switches) was computed from the
real `dataconstants.h` MixSources enum and cross-checked against a known-good
profile (sticks decoded as Rud=75..Ail=78, mixes referencing I1..I4).

Safety
------
Format is board/version specific. On load the tool:
  1. checks the 'otx' fourcc, board byte '3' (X9D) and version 219, and
  2. self-checks: it re-encodes every mix/expo entry it parsed and asserts the
     bytes equal the original. If your bitfield model were wrong this fails loudly
     instead of silently corrupting a model that flies (or drives) hardware.
If either check fails the tool refuses to operate on the file.

CLI
---
    otx_tool.py dump      file.otx [--model N]
    otx_tool.py verify    file.otx
    otx_tool.py set-expo  file.otx --chn C --value V [--model N] [-o out|--in-place]
    otx_tool.py add-mix   file.otx --ch CH --src SRC [--weight W] [--mltpx add]
                          [--switch S] [--trim-off] [--model N] [-o out|--in-place]

`src`/`switch` accept either a raw number or a name ("TR1", "I4", "Ele", "SA").
In-place edits write a .bak backup first.
"""

from __future__ import annotations

import argparse
import io
import os
import shutil
import sys
import zipfile
from dataclasses import dataclass

# --- file/struct layout (PCBX9D, EEPROM_VER 219) ---------------------------
FOURCC = b"otx3"  # 'otx' + board byte '3' == Taranis X9D family (incl. X9D+ 2019)
EEPROM_VER = 219
HDR = 8  # fourcc(4) + ver(1) + 'M'(1) + size(uint16)
MODEL_STRUCT_SIZE = 6604  # sizeof(ModelData) for this board/version (uncompressed)

# struct-relative offsets; file offset = HDR + these
MIX_OFF, MIX_SZ, MIX_N = 76, 20, 64
LIM_OFF, LIM_SZ, LIM_N = 1356, 13, 32
EXP_OFF, EXP_SZ, EXP_N = 1772, 17, 64

# CurveRef.type
CURVE_REF_DIFF, CURVE_REF_EXPO, CURVE_REF_FUNC, CURVE_REF_CUSTOM = 0, 1, 2, 3
# MixData.mltpx
MLTPX_ADD, MLTPX_MUL, MLTPX_REP = 0, 1, 2
MLTPX_NAMES = {MLTPX_ADD: "+=", MLTPX_MUL: "*=", MLTPX_REP: ":="}
MLTPX_FROM_STR = {
    "add": MLTPX_ADD,
    "mul": MLTPX_MUL,
    "rep": MLTPX_REP,
    "replace": MLTPX_REP,
    "+=": MLTPX_ADD,
    "*=": MLTPX_MUL,
    ":=": MLTPX_REP,
}
# MixData.carry_trim: 0 == trim applied (TRIM_ON), 1 == trim off (TRIM_OFF)
TRIM_ON, TRIM_OFF = 0, 1

# --- MIXSRC source numbering (X9D+ 2019), used only for human-readable names -
# IMPORTANT: this is the Taranis X9D+ 2019 (BOARD_NAME=x9d+2019), which has 9
# switches SA..SI (SI is a second momentary switch). The plain X9D has only 8
# (SA..SH). Both share the 'otx3' fourcc, so the file alone does NOT tell you
# which: the firmware build does. The 9th switch shifts every source AFTER the
# switch block up by one vs a plain X9D, so on this board:
#     SI = 100, FIRST_LOGICAL_SWITCH = 101, FIRST_TRAINER (TR1) = 165
# (On a plain X9D those would be: no SI, 100, 164.) Values verified against a
# real SD model where CH7 was mapped to SI and decoded as raw 100.
MIXSRC_FIRST_INPUT = 1  # I1..I32
MIXSRC_FIRST_STICK = 75  # Rud,Ele,Thr,Ail
MIXSRC_FIRST_POT = 79
MIXSRC_FIRST_SWITCH = 92  # SA..SI (9 switches)
MIXSRC_FIRST_LOGICAL_SWITCH = 101
MIXSRC_FIRST_TRAINER = 165  # TR1..TR16
STICK_NAMES = ["Rud", "Ele", "Thr", "Ail"]
POT_NAMES = ["S1", "S2", "S3", "LS", "RS"]
SWITCH_LETTERS = "ABCDEFGHI"


def src_name(v: int) -> str:
    if v == 0:
        return "----"
    if MIXSRC_FIRST_INPUT <= v <= MIXSRC_FIRST_INPUT + 31:
        return f"I{v - MIXSRC_FIRST_INPUT + 1}"
    if MIXSRC_FIRST_STICK <= v <= MIXSRC_FIRST_STICK + 3:
        return STICK_NAMES[v - MIXSRC_FIRST_STICK]
    if MIXSRC_FIRST_POT <= v < MIXSRC_FIRST_SWITCH:
        i = v - MIXSRC_FIRST_POT
        return POT_NAMES[i] if i < len(POT_NAMES) else f"POT{i}"
    if MIXSRC_FIRST_SWITCH <= v <= MIXSRC_FIRST_SWITCH + len(SWITCH_LETTERS) - 1:
        return f"S{SWITCH_LETTERS[v - MIXSRC_FIRST_SWITCH]}"
    if MIXSRC_FIRST_TRAINER <= v <= MIXSRC_FIRST_TRAINER + 15:
        return f"TR{v - MIXSRC_FIRST_TRAINER + 1}"
    if MIXSRC_FIRST_LOGICAL_SWITCH <= v < MIXSRC_FIRST_TRAINER:
        return f"L{v - MIXSRC_FIRST_LOGICAL_SWITCH + 1}"
    return f"src{v}"


def _src_from_prefix(up: str):
    """Parse a prefixed source name (TR1, I4, L2, SA). Returns None if no match."""
    if up.startswith("TR"):
        return MIXSRC_FIRST_TRAINER + int(up[2:]) - 1
    if up.startswith("I") and up[1:].isdigit():
        return MIXSRC_FIRST_INPUT + int(up[1:]) - 1
    if up.startswith("L") and up[1:].isdigit():
        return MIXSRC_FIRST_LOGICAL_SWITCH + int(up[1:]) - 1
    if up.startswith("S") and len(up) == 2 and up[1] in SWITCH_LETTERS:
        return MIXSRC_FIRST_SWITCH + SWITCH_LETTERS.index(up[1])
    return None


def _src_from_table(up: str):
    """Parse a stick or pot name (Ele, S1). Returns None if no match."""
    for i, n in enumerate(STICK_NAMES):
        if up == n.upper():
            return MIXSRC_FIRST_STICK + i
    for i, n in enumerate(POT_NAMES):
        if up == n.upper():
            return MIXSRC_FIRST_POT + i
    return None


def src_value(token) -> int:
    """Parse a source given as a number or a name (TR1, I4, Ele, SA, S1)."""
    if isinstance(token, int):
        return token
    t = str(token).strip()
    if t.lstrip("-").isdigit():
        return int(t)
    up = t.upper()
    v = _src_from_prefix(up)
    if v is None:
        v = _src_from_table(up)
    if v is None:
        raise ValueError(f"cannot parse source {token!r}")
    return v


# --- packed little-endian bitfield codec -----------------------------------
# Both MixData and ExpoData begin with exactly 64 bits of bitfields (a whole
# 8-byte run) followed by byte-aligned members. GCC packs bitfields LSB-first on
# little-endian, so the first declared field is the least significant bits.


def _decode_bits(b8: bytes, specs):
    val = int.from_bytes(b8, "little")
    out, pos = {}, 0
    for name, width, signed in specs:
        f = (val >> pos) & ((1 << width) - 1)
        if signed and (f >> (width - 1)):
            f -= 1 << width
        out[name] = f
        pos += width
    assert pos == 64, pos
    return out


def _encode_bits(fields, specs) -> bytes:
    val, pos = 0, 0
    for name, width, signed in specs:
        f = fields[name] & ((1 << width) - 1)
        val |= f << pos
        pos += width
    assert pos == 64, pos
    return val.to_bytes(8, "little")


MIX_SPECS = [
    ("weight", 11, True),
    ("dest_ch", 5, False),
    ("src_raw", 10, False),
    ("carry_trim", 1, False),
    ("mix_warn", 2, False),
    ("mltpx", 2, False),
    ("spare", 1, False),
    ("offset", 14, True),
    ("swtch", 9, True),
    ("flight_modes", 9, False),
]
EXP_SPECS = [
    ("mode", 2, False),
    ("scale", 14, False),
    ("src_raw", 10, False),
    ("trim_source", 6, True),
    ("chn", 5, False),
    ("swtch", 9, True),
    ("flight_modes", 9, False),
    ("weight", 8, True),
    ("spare", 1, True),
]


def _s8(v):  # signed byte helper
    return v - 256 if v > 127 else v


# --- OpenTX RLE (RLC) codec for radio SD-card model files -------------------
# Mirrors radio/src/storage/eeprom_rlc.cpp (readRlc / nextRlcWriteStep). Control
# byte: 0x80 set -> (bits4-6)=zeros then (bits0-3)=literal count; 0x40 set ->
# (bits0-5)=zero count; otherwise -> literal count (1..0x3f). A 0x00 control byte
# ends the stream (the rest of the struct is zeros). The compressor reproduces
# the radio's exact output (verified byte-for-byte on a real SD model).


def rlc_decompress(data: bytes, out_len: int) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while len(out) < out_len and i < n:
        c = data[i]
        i += 1
        if c == 0:
            break
        if c & 0x80:
            lit, zer = c & 0x0F, (c >> 4) & 0x7
        elif c & 0x40:
            lit, zer = 0, c & 0x3F
        else:
            lit, zer = c, 0
        litb = data[i : i + lit]
        i += lit
        out += b"\x00" * zer
        out += litb
    out += b"\x00" * (out_len - len(out))
    return bytes(out[:out_len])


def rlc_compress(data: bytes) -> bytes:
    out = bytearray()
    end = len(data)
    while end > 0 and data[end - 1] == 0:  # radio stores only up to last nonzero
        end -= 1
    i, pend = 0, 0
    while i < end:
        if data[i] == 0:
            z = 0
            while i < end and data[i] == 0 and z < 0x3F:
                z += 1
                i += 1
            if z < 8 and i < end:  # small run before a literal: defer
                pend = z
            else:
                out.append(z | 0x40)
        else:
            cap = 0x0F if pend else 0x3F
            lit = bytearray()
            while i < end and data[i] != 0 and len(lit) < cap:
                lit.append(data[i])
                i += 1
            out.append((0x80 | (pend << 4) | len(lit)) if pend else len(lit))
            out += lit
            pend = 0
    return bytes(out)


@dataclass
class Mix:
    weight: int = 0
    dest_ch: int = 0
    src_raw: int = 0
    carry_trim: int = 0
    mix_warn: int = 0
    mltpx: int = 0
    spare: int = 0
    offset: int = 0
    swtch: int = 0
    flight_modes: int = 0
    curve_type: int = 0
    curve_value: int = 0
    delay_up: int = 0
    delay_down: int = 0
    speed_up: int = 0
    speed_down: int = 0
    name: bytes = b"\x00" * 6

    @classmethod
    def from_bytes(cls, b: bytes) -> "Mix":
        f = _decode_bits(b[:8], MIX_SPECS)
        return cls(
            **f,
            curve_type=b[8],
            curve_value=_s8(b[9]),
            delay_up=b[10],
            delay_down=b[11],
            speed_up=b[12],
            speed_down=b[13],
            name=bytes(b[14:20]),
        )

    def to_bytes(self) -> bytes:
        head = _encode_bits(self.__dict__, MIX_SPECS)
        tail = bytes(
            [
                self.curve_type,
                self.curve_value & 0xFF,
                self.delay_up,
                self.delay_down,
                self.speed_up,
                self.speed_down,
            ]
        )
        nm = (self.name + b"\x00" * 6)[:6]
        out = head + tail + nm
        assert len(out) == MIX_SZ
        return out

    def empty(self) -> bool:
        return self.src_raw == 0 and self.dest_ch == 0 and self.weight == 0

    def describe(self) -> str:
        sw = f" sw={self.swtch}" if self.swtch else ""
        cv = (
            f" expo/diff({self.curve_type},{self.curve_value})"
            if self.curve_type or self.curve_value
            else ""
        )
        trim = "" if self.carry_trim == TRIM_ON else " trimOFF"
        return (
            f"CH{self.dest_ch + 1} {MLTPX_NAMES.get(self.mltpx, '?')} "
            f"{src_name(self.src_raw)} w={self.weight}{sw} off={self.offset}{cv}{trim}"
        )


@dataclass
class Expo:
    mode: int = 0
    scale: int = 0
    src_raw: int = 0
    trim_source: int = 0
    chn: int = 0
    swtch: int = 0
    flight_modes: int = 0
    weight: int = 0
    spare: int = 0
    name: bytes = b"\x00" * 6
    offset: int = 0
    curve_type: int = 0
    curve_value: int = 0

    @classmethod
    def from_bytes(cls, b: bytes) -> "Expo":
        f = _decode_bits(b[:8], EXP_SPECS)
        return cls(
            **f, name=bytes(b[8:14]), offset=_s8(b[14]), curve_type=b[15], curve_value=_s8(b[16])
        )

    def to_bytes(self) -> bytes:
        head = _encode_bits(self.__dict__, EXP_SPECS)
        nm = (self.name + b"\x00" * 6)[:6]
        tail = bytes([self.offset & 0xFF, self.curve_type, self.curve_value & 0xFF])
        out = head + nm + tail
        assert len(out) == EXP_SZ
        return out

    def empty(self) -> bool:
        return self.src_raw == 0 and self.chn == 0 and self.weight == 0

    def describe(self) -> str:
        sw = f" sw={self.swtch}" if self.swtch else ""
        if self.curve_type == CURVE_REF_EXPO:
            cv = f" expo={self.curve_value}%"
        elif self.curve_type == CURVE_REF_DIFF and self.curve_value:
            cv = f" diff={self.curve_value}%"
        elif self.curve_type or self.curve_value:
            cv = f" curve({self.curve_type},{self.curve_value})"
        else:
            cv = " linear"
        return f"I{self.chn + 1} <- {src_name(self.src_raw)} w={self.weight}{cv}{sw}"


class Model:
    """One modelN.bin: full raw bytes plus decoded mix/expo arrays."""

    def __init__(self, raw: bytes, name_in_zip: str):
        self.raw = bytearray(raw)
        self.name_in_zip = name_in_zip
        if bytes(self.raw[0:4]) != FOURCC:
            raise ValueError(
                f"{name_in_zip}: bad fourcc {bytes(self.raw[0:4])!r} "
                f"(expected {FOURCC!r}, X9D family)"
            )
        self.version = self.raw[4]
        if self.version != EEPROM_VER or self.raw[5] != ord("M"):
            raise ValueError(
                f"{name_in_zip}: unsupported version {self.version}"
                f"/{chr(self.raw[5])}; tool targets v{EEPROM_VER}"
            )
        self.mixes = [
            Mix.from_bytes(self._slice(MIX_OFF + i * MIX_SZ, MIX_SZ)) for i in range(MIX_N)
        ]
        self.expos = [
            Expo.from_bytes(self._slice(EXP_OFF + i * EXP_SZ, EXP_SZ)) for i in range(EXP_N)
        ]
        self._self_check()

    def _slice(self, struct_off: int, n: int) -> bytes:
        a = HDR + struct_off
        return bytes(self.raw[a : a + n])

    def _self_check(self):
        """Re-encode parsed entries; they must reproduce the original bytes."""
        for i, m in enumerate(self.mixes):
            if m.to_bytes() != self._slice(MIX_OFF + i * MIX_SZ, MIX_SZ):
                raise AssertionError(
                    f"{self.name_in_zip}: mix[{i}] codec mismatch (layout wrong, refusing)"
                )
        for i, e in enumerate(self.expos):
            if e.to_bytes() != self._slice(EXP_OFF + i * EXP_SZ, EXP_SZ):
                raise AssertionError(
                    f"{self.name_in_zip}: expo[{i}] codec mismatch (layout wrong, refusing)"
                )

    @property
    def model_name(self) -> str:
        return self.raw[HDR : HDR + 12].split(b"\x00")[0].decode("latin1")

    # --- editing ---
    def set_input_expo(self, chn: int, value: int) -> int:
        """Set EXPO curve on every input line for input `chn` (0-based). Returns count."""
        n = 0
        for e in self.expos:
            if e.src_raw != 0 and e.chn == chn:
                e.curve_type = CURVE_REF_EXPO
                e.curve_value = value
                n += 1
        if n == 0:
            raise ValueError(f"no input lines found for I{chn + 1}")
        return n

    def set_mix_src(self, ch: int, new_src: int, old_src=None) -> int:
        """Repoint mix line(s) on channel `ch` (0-based) to `new_src`. If
        `old_src` is given, only lines currently using it are changed. Returns count."""
        n = 0
        for m in self.mixes:
            if not m.empty() and m.dest_ch == ch and (old_src is None or m.src_raw == old_src):
                m.src_raw = new_src
                n += 1
        if n == 0:
            raise ValueError(f"no matching mix line on CH{ch + 1}")
        return n

    def _mix_count(self) -> int:
        last = 0
        for i, m in enumerate(self.mixes):
            if not m.empty():
                last = i + 1
        return last

    def add_mix(
        self,
        ch: int,
        src: int,
        weight: int = 100,
        mltpx: int = MLTPX_ADD,
        switch: int = 0,
        trim_off: bool = True,
    ) -> int:
        """Insert a mix line on channel `ch` (0-based), after that channel's last
        existing line, shifting subsequent mixes down. Returns the insert index."""
        if self._mix_count() >= MIX_N:
            raise ValueError("mix table full")
        last = -1
        for i in range(self._mix_count()):
            if self.mixes[i].dest_ch == ch:
                last = i
        idx = last + 1 if last >= 0 else self._mix_count()
        m = Mix(
            weight=weight,
            dest_ch=ch,
            src_raw=src,
            mltpx=mltpx,
            swtch=switch,
            carry_trim=(TRIM_OFF if trim_off else TRIM_ON),
        )
        self.mixes.insert(idx, m)
        self.mixes = self.mixes[:MIX_N]
        return idx

    def serialize(self) -> bytes:
        """Write mix/expo arrays back into raw; all other bytes untouched."""
        for i, m in enumerate(self.mixes):
            a = HDR + MIX_OFF + i * MIX_SZ
            self.raw[a : a + MIX_SZ] = m.to_bytes()
        for i, e in enumerate(self.expos):
            a = HDR + EXP_OFF + i * EXP_SZ
            self.raw[a : a + EXP_SZ] = e.to_bytes()
        return bytes(self.raw)


class Otx:
    """An .otx ZIP. Preserves every entry; only rewrites edited model bins."""

    def __init__(self, path: str):
        self.path = path
        self.entries: dict[str, bytes] = {}
        self.order: list[str] = []
        with zipfile.ZipFile(path, "r") as z:
            for info in z.infolist():
                self.order.append(info.filename)
                self.entries[info.filename] = z.read(info.filename)
        self.model_files = sorted(
            n for n in self.order if n.startswith("MODELS/") and n.endswith(".bin")
        )
        self.models = [Model(self.entries[n], n) for n in self.model_files]

    def save(self, out_path: str):
        for m in self.models:
            self.entries[m.name_in_zip] = m.serialize()
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for name in self.order:
                z.writestr(name, self.entries[name])
        with open(out_path, "wb") as f:
            f.write(buf.getvalue())

    def self_test(self) -> str:
        for m in self.models:
            if m.serialize() != bytes(m.raw):
                raise AssertionError(f"{m.name_in_zip} not stable under serialize")
        return f"{len(self.models)} model(s), codec self-check + round-trip passed"


class SdBin:
    """A single radio SD-card model .bin file (MODELS/<name>.bin).

    On EEPROM_RLC builds the body is RLE-compressed; on EEPROM_SDCARD builds it is
    a raw struct dump. We discriminate on the header size field (== full struct
    size means uncompressed). Decompress on load, recompress on save."""

    def __init__(self, path: str):
        self.path = path
        raw = open(path, "rb").read()
        if raw[:4] != FOURCC:
            raise ValueError(f"{path}: bad fourcc {raw[:4]!r} (expected {FOURCC!r}, X9D family)")
        self._orig = raw
        size = int.from_bytes(raw[6:8], "little")
        body = raw[8:]
        if size == MODEL_STRUCT_SIZE and len(body) >= MODEL_STRUCT_SIZE:
            self.compressed = False
            struct = body[:MODEL_STRUCT_SIZE]
        else:
            self.compressed = True
            struct = rlc_decompress(body, MODEL_STRUCT_SIZE)
        self.models = [Model(raw[:HDR] + struct, os.path.basename(path))]

    def save(self, out_path: str):
        m = self.models[0]
        struct = m.serialize()[HDR : HDR + MODEL_STRUCT_SIZE]
        body = rlc_compress(struct) if self.compressed else struct
        hdr = bytearray(m.raw[:HDR])
        hdr[6:8] = len(body).to_bytes(2, "little")
        with open(out_path, "wb") as f:
            f.write(bytes(hdr) + body)

    def self_test(self) -> str:
        m = self.models[0]
        if m.serialize() != bytes(m.raw):
            raise AssertionError("model not stable under serialize")
        if self.compressed:
            # the RLE codec must reproduce the radio's exact stored bytes
            struct = m.serialize()[HDR : HDR + MODEL_STRUCT_SIZE]
            if rlc_compress(struct) != self._orig[HDR:]:
                raise AssertionError("RLE recompression does not match original bytes")
        return f"1 model ({'RLE' if self.compressed else 'raw'}), codec + RLE round-trip passed"


def open_container(path: str):
    """Return an editable container for either a .otx ZIP or a single SD .bin."""
    if zipfile.is_zipfile(path):
        return Otx(path)
    return SdBin(path)


# --- CLI -------------------------------------------------------------------
def _pick(otx: Otx, idx):
    if idx is None:
        if len(otx.models) == 1:
            return otx.models[0]
        return otx.models  # caller handles list
    return otx.models[idx]


def _print_model(m: Model, label: str):
    print(f"== {label} ({m.name_in_zip}, name={m.model_name!r}) ==")
    print(" Inputs:")
    for e in m.expos:
        if not e.empty():
            print("   " + e.describe())
    print(" Mixes:")
    for x in m.mixes:
        if not x.empty():
            print("   " + x.describe())


def cmd_dump(args):
    c = open_container(args.file)
    sel = range(len(c.models)) if args.model is None else [args.model]
    for i in sel:
        _print_model(c.models[i], f"model[{i}]")


def cmd_verify(args):
    c = open_container(args.file)  # construction self-checks every model
    print(f"OK: {args.file} - fourcc/version valid, {c.self_test()}")
    return 0


def _write_out(container, args):
    if args.in_place:
        bak = args.file + ".bak"
        if not os.path.exists(bak):
            shutil.copy2(args.file, bak)
            print(f"backup: {bak}")
        container.save(args.file)
        print(f"wrote: {args.file}")
    else:
        base, ext = os.path.splitext(args.file)
        out = args.out or (base + "_edited" + ext)
        container.save(out)
        print(f"wrote: {out}")


def cmd_set_expo(args):
    c = open_container(args.file)
    m = c.models[args.model or 0]
    n = m.set_input_expo(args.chn, args.value)
    print(f"set expo={args.value}% on {n} line(s) of I{args.chn + 1}")
    _write_out(c, args)


def cmd_add_mix(args):
    c = open_container(args.file)
    m = c.models[args.model or 0]
    src = src_value(args.src)
    sw = src_value(args.switch) if args.switch else 0
    idx = m.add_mix(
        args.ch,
        src,
        weight=args.weight,
        mltpx=MLTPX_FROM_STR[args.mltpx],
        switch=sw,
        trim_off=args.trim_off,
    )
    print(f"inserted mix at [{idx}]: CH{args.ch + 1} {args.mltpx} {src_name(src)} w={args.weight}")
    _write_out(c, args)


def cmd_set_mix_src(args):
    c = open_container(args.file)
    m = c.models[args.model or 0]
    new = src_value(args.src)
    old = src_value(getattr(args, "from")) if getattr(args, "from") else None
    n = m.set_mix_src(args.ch, new, old_src=old)
    frm = f" (from {src_name(old)})" if old is not None else ""
    print(f"repointed {n} mix line(s) on CH{args.ch + 1}{frm} -> {src_name(new)}")
    _write_out(c, args)


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Read/edit OpenTX model files (.otx ZIP or SD .bin), Taranis X9D+ 2019, v219"
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("dump", help="print inputs and mixes")
    d.add_argument("file")
    d.add_argument("--model", type=int)
    d.set_defaults(func=cmd_dump)

    v = sub.add_parser("verify", help="validate format + codec self-check")
    v.add_argument("file")
    v.set_defaults(func=cmd_verify)

    e = sub.add_parser("set-expo", help="set EXPO curve on an input")
    e.add_argument("file")
    e.add_argument("--chn", type=int, required=True, help="0-based input index (I1=0)")
    e.add_argument("--value", type=int, required=True, help="expo percent, e.g. 50")
    e.add_argument("--model", type=int)
    e.add_argument("-o", "--out")
    e.add_argument("--in-place", action="store_true")
    e.set_defaults(func=cmd_set_expo)

    a = sub.add_parser("add-mix", help="insert a mix line on a channel")
    a.add_argument("file")
    a.add_argument("--ch", type=int, required=True, help="0-based channel (CH1=0)")
    a.add_argument("--src", required=True, help="source: number or name (TR1, I4, Ele, SA)")
    a.add_argument("--weight", type=int, default=100)
    a.add_argument("--mltpx", choices=list(MLTPX_FROM_STR), default="add")
    a.add_argument(
        "--switch", default=None, help="enable switch: number or name (default none/always on)"
    )
    a.add_argument("--trim-off", action="store_true", help="do not carry stick trim into this line")
    a.add_argument("--model", type=int)
    a.add_argument("-o", "--out")
    a.add_argument("--in-place", action="store_true")
    a.set_defaults(func=cmd_add_mix)

    s = sub.add_parser("set-mix-src", help="repoint an existing channel's mix source")
    s.add_argument("file")
    s.add_argument("--ch", type=int, required=True, help="0-based channel (CH1=0)")
    s.add_argument("--src", required=True, help="new source: number or name (SI, TR1, Ail)")
    s.add_argument("--from", default=None, help="only change lines currently using this source")
    s.add_argument("--model", type=int)
    s.add_argument("-o", "--out")
    s.add_argument("--in-place", action="store_true")
    s.set_defaults(func=cmd_set_mix_src)

    args = p.parse_args(argv)
    return args.func(args) or 0


if __name__ == "__main__":
    sys.exit(main())
