"""BLHeli_32 settings codec and 4-way client, against pages read from Mr Stabs Mk2's ESCs."""

from __future__ import annotations

from pathlib import Path

import pytest

from auto_battlebot.blheli32 import cipher, layout
from auto_battlebot.blheli32.esc import Session, VerifyError
from auto_battlebot.blheli32.fourway import (
    CMD_DEVICE_INIT_FLASH,
    CMD_DEVICE_READ,
    CMD_DEVICE_WRITE,
    START_RESPONSE,
    build_request,
    crc16_xmodem,
    parse_response,
)

CAPTURES = Path(__file__).resolve().parents[2] / "firmware/config/blheli/captures"


def page(esc: int, addr: str = "f800") -> bytes:
    return (CAPTURES / f"mr_stabs_mk2_esc{esc}_{addr}_2026-10-23.bin").read_bytes()


# What BLHeliSuite32xl showed for ESC 1 after Read Setup on 2026-10-23.
BLHELISUITE_ESC1 = {
    "direction": 3,
    "rampup_power": 100,
    "pwm_frequency_low": 24,
    "comm_timing": 16,
    "demag_compensation": 3,
    "min_throttle": 1000,
    "center_throttle": 1500,
    "max_throttle": 2000,
    "throttle_cal_enable": 1,
    "temperature_protection": 140,
    "low_voltage_protection": 0,
    "low_rpm_power_protect": 0,
    "brake_on_stop": 100,
    "startup_beep_volume": 254,
    "beacon_volume": 80,
    "beacon_delay": 600,
    "max_acceleration": 0,
    "nondamped_mode": 0,
    "sine_mode": 0,
    "auto_telemetry": 1,
    "stall_protection": 0,
    "sbus_channel": 17,
    "sport_physical_id": 0,
    "pwm_frequency_high": 24,
}


@pytest.mark.parametrize("esc", [1, 2])
def test_page_round_trips(esc: int) -> None:
    p = page(esc)
    nonces = cipher.page_nonces(p)
    assert cipher.encode_page(cipher.decode_page(p), nonce=lambda n: nonces[n]) == p


def test_nonces_follow_the_esc_pattern() -> None:
    nonces = cipher.page_nonces(page(1))
    assert cipher.esc_style_nonces(nonces[0])(31) == nonces[31]


def test_fresh_nonces_change_ciphertext_not_payload() -> None:
    payload = cipher.decode_page(page(1))
    a, b = cipher.encode_page(payload), cipher.encode_page(payload)
    assert a != b
    assert cipher.decode_page(a) == cipher.decode_page(b) == payload


def test_decode_matches_blhelisuite() -> None:
    settings = layout.decode(cipher.decode_page(page(1)))
    assert {k: settings[k] for k in BLHELISUITE_ESC1} == BLHELISUITE_ESC1
    assert str(settings["esc_layout"]).startswith("#Flycolor_Raptor_5#")
    assert str(settings["esc_mcu"]).startswith("#BLHeli_32*STM32G071x6#")


def test_escs_differ_only_in_direction_and_one_music_byte() -> None:
    a, b = (cipher.decode_page(page(e)) for e in (1, 2))
    assert [i for i in range(len(a)) if a[i] != b[i]] == [3, 144]
    assert (a[3], b[3]) == (3, 4)  # Bidirectional 3D, Bidirectional 3D Rev.


def test_wrong_page_length_is_rejected() -> None:
    with pytest.raises(cipher.PageError):
        cipher.decode_page(page(1)[:255])


def test_encode_changes_only_the_field() -> None:
    before = cipher.decode_page(page(1))
    after = layout.encode(before, {"demag_compensation": 4, "beacon_delay": 1200})
    assert [i for i in range(len(before)) if before[i] != after[i]] == [7, 22, 23]
    assert layout.decode(after)["beacon_delay"] == 1200


@pytest.mark.parametrize(
    "changes",
    [
        {"rampup_power": 2},
        {"demag_compensation": 0},
        {"min_throttle": 1600},
        {"pwm_frequency_low": 40},  # above pwm_frequency_high (24)
        {"fw_main_revision": 33},
        {"esc_layout": 1},
    ],
)
def test_encode_rejects_invalid(changes: dict[str, int]) -> None:
    with pytest.raises(layout.LayoutError):
        layout.encode(cipher.decode_page(page(1)), changes)


def test_parse_value_accepts_labels() -> None:
    assert layout.parse_value("demag_compensation", "Very High") == 4
    assert layout.parse_value("comm_timing", "Auto") == 0
    assert layout.parse_value("pwm_frequency_high", "By RPM") == 129
    assert layout.parse_value("brake_on_stop", "off") == 0
    assert layout.parse_value("rampup_power", "60") == 60


def test_wrong_layout_is_refused() -> None:
    payload = bytearray(cipher.decode_page(page(1)))
    payload[2] = 47
    with pytest.raises(layout.LayoutError):
        layout.decode(bytes(payload))


def response(cmd: int, address: int, params: bytes, ack: int = 0) -> bytes:
    body = bytes([START_RESPONSE, cmd, address >> 8, address & 0xFF, len(params) & 0xFF]) + params
    body += bytes([ack])
    return body + crc16_xmodem(body).to_bytes(2, "big")


def test_request_framing() -> None:
    req = build_request(CMD_DEVICE_READ, 0xF800, b"\x00")
    assert req[:6] == bytes([0x2F, 0x3A, 0xF8, 0x00, 0x01, 0x00])
    assert crc16_xmodem(req[:-2]) == int.from_bytes(req[-2:], "big")
    r = parse_response(response(CMD_DEVICE_INIT_FLASH, 0, bytes.fromhex("06476d04")))
    assert r.params == bytes.fromhex("06476d04")


class FakePassthrough:
    """Answers 4-way requests like the passthrough and an ESC, from the captured ESC 1 data.

    The ESC re-encrypts on every read with fresh nonces, as the real one does. drop_writes
    acknowledges writes without storing them, like the dry-run passthrough build.
    """

    def __init__(self, drop_writes: bool = False) -> None:
        self.rx = bytearray()
        self.drop_writes = drop_writes
        self.settings = cipher.decode_page(page(1))
        self.memory = {0xFA00: page(1, "fa00"), 0xFD90: page(1, "fd90")}
        self.writes: list[tuple[int, bytes]] = []

    def write(self, data: bytes) -> int:
        if data.startswith(b"$M<"):
            self.rx += b"$M>\x00\xf5\xf5"
            return len(data)
        cmd, addr = data[1], (data[2] << 8) | data[3]
        if cmd == CMD_DEVICE_READ and addr == cipher.PAGE_ADDRESS:
            self.rx += response(cmd, addr, cipher.encode_page(self.settings)[: data[5] or 256])
        elif cmd == CMD_DEVICE_WRITE:
            params = data[5 : 5 + (data[4] or 256)]
            self.writes.append((addr, params))
            if not self.drop_writes:
                self.settings = cipher.decode_page(params)
            self.rx += response(cmd, addr, b"\x00")
        elif cmd == CMD_DEVICE_READ:
            n = data[5] or 256
            self.rx += response(cmd, addr, self.memory[addr][:n])
        elif cmd == 0x32:
            self.rx += response(cmd, 0, b"m4wFCIntf")
        elif cmd == CMD_DEVICE_INIT_FLASH:
            self.rx += response(cmd, 0, bytes.fromhex("06476d04"))
        else:
            self.rx += response(cmd, addr, b"\x00")
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out, self.rx = bytes(self.rx[:size]), self.rx[size:]
        return out

    def reset_input_buffer(self) -> None:
        self.rx.clear()


def test_session_reads() -> None:
    session = Session(FakePassthrough())
    assert session.open() == "m4wFCIntf"
    snap = session.read(0)
    assert snap.good_frames == 338422
    assert snap.input_protocol == 5
    assert layout.decode(snap.payload)["rampup_power"] == 100


def test_write_is_one_page_and_verifies() -> None:
    esc = FakePassthrough()
    session = Session(esc)
    session.open()
    before = session.read(0).payload
    edited = layout.encode(before, {"pwm_frequency_low": 16, "pwm_frequency_high": 48})
    after = session.write(0, edited, expect_current=before)
    assert after.payload == edited
    # BLHeliSuite's Write Setup: a single 256-byte DeviceWrite at 0xF800, no page erase.
    assert [(a, len(p)) for a, p in esc.writes] == [(0xF800, 256)]


def test_write_of_unchanged_settings_sends_nothing() -> None:
    esc = FakePassthrough()
    session = Session(esc)
    session.open()
    session.write(0, session.read(0).payload)
    assert esc.writes == []


def test_dropped_write_fails_verify() -> None:
    session = Session(FakePassthrough(drop_writes=True))
    session.open()
    before = session.read(0).payload
    with pytest.raises(VerifyError, match=r"\[5\]"):
        session.write(0, layout.encode(before, {"pwm_frequency_low": 16}))


def test_stale_backup_is_refused_before_writing() -> None:
    esc = FakePassthrough()
    session = Session(esc)
    session.open()
    before = session.read(0).payload
    stale = layout.encode(before, {"rampup_power": 50})
    with pytest.raises(VerifyError, match="no longer matches"):
        session.write(0, layout.encode(before, {"brake_on_stop": 0}), expect_current=stale)
    assert esc.writes == []


def test_blhelisuite_write_setup_page() -> None:
    """The page BLHeliSuite wrote with PWM low 16 / high 48 differs only at bytes 5 and 34."""
    before = cipher.decode_page(page(1))
    ours = layout.encode(before, {"pwm_frequency_low": 16, "pwm_frequency_high": 48})
    assert [i for i in range(len(before)) if before[i] != ours[i]] == [5, 34]


def test_music_codec_matches_blhelisuite() -> None:
    """BLHeliSuite encoded this script as 10 51 A2 80 DB 19 in a captured Write Setup."""
    payload = cipher.decode_page(page(1))
    assert layout.decode_music(payload) == "A5 1/8"
    assert layout.decode_music(cipher.decode_page(page(2))) == "D5 1/8"
    script = "C5 1/8 C#5 1/4 D6 1/2 C4 1/2 B5 1/1 A5 1/8"
    music = layout.encode_music(payload, script)
    assert music[144:151] == bytes.fromhex("1051a280db19ff")
    assert layout.decode_music(music) == script
    with pytest.raises(layout.LayoutError):
        layout.encode_music(payload, "C4 1/16")


def test_name_is_space_padded() -> None:
    payload = layout.encode_name(cipher.decode_page(page(1)), "LEFT")
    assert payload[128:144] == b"LEFT".ljust(16)
    with pytest.raises(layout.LayoutError):
        layout.encode_name(payload, "x" * 17)
