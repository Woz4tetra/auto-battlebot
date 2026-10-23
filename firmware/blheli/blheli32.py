#!/usr/bin/env python3
"""Read, back up, and edit BLHeli_32 ESC settings through the 4-way passthrough.

The robot runs firmware/BlHeli-Passthrough. ESC 0 is pin A2, ESC 1 is pin A3.

    venv/bin/python firmware/blheli/blheli32.py read                    # print both ESCs
    venv/bin/python firmware/blheli/blheli32.py backup out/             # save pages + decoded JSON
    venv/bin/python firmware/blheli/blheli32.py show backup.json        # print a saved backup
    venv/bin/python firmware/blheli/blheli32.py set backup.json demag_compensation=High \
        rampup_power=60 -o new.json                                     # edit offline, validates
    venv/bin/python firmware/blheli/blheli32.py set backup.json music_notes="A5 1/8 D5 1/4" \
        name=LEFT -o new.json                                           # startup tune and name
    venv/bin/python firmware/blheli/blheli32.py write new.json --esc 0  # diff, confirm, write
    venv/bin/python firmware/blheli/blheli32.py fields                  # every field and range

Field reference: firmware/blheli/README.md.
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from pathlib import Path
from typing import Any

from auto_battlebot.blheli32 import cipher, layout
from auto_battlebot.blheli32.esc import EscSnapshot, Session

DEFAULT_PORT = "/dev/mr_stabs"


def open_session(port: str) -> tuple[Session, Any]:
    import serial

    ser = serial.Serial(port, 115200, timeout=0.2)
    session = Session(ser)
    name = session.open()
    print(f"interface: {name}", file=sys.stderr)
    return session, ser


def snapshot_json(s: EscSnapshot) -> dict[str, Any]:
    return {
        "esc": s.index,
        "read_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "signature": s.signature.hex(),
        "page_b64": base64.b64encode(s.page).decode(),
        "payload_hex": s.payload.hex(),
        "stats_hex": s.stats.hex(),
        "info_hex": s.info.hex(),
        "good_frames": s.good_frames,
        "bad_frames": s.bad_frames,
        "settings": layout.decode(s.payload),
    }


def payload_from_json(doc: dict[str, Any]) -> bytes:
    payload = bytes.fromhex(doc["payload_hex"])
    if "page_b64" in doc and cipher.decode_page(base64.b64decode(doc["page_b64"])) != payload:
        raise SystemExit("backup is inconsistent: page_b64 does not decode to payload_hex")
    return payload


def show_text(f: layout.Field, payload: bytes) -> str:
    if f.kind == "music":
        return layout.decode_music(payload)
    return payload[f.offset : f.offset + f.width].decode("latin-1").rstrip(" \xff")


def print_settings(payload: bytes, title: str, compare: bytes | None = None) -> None:
    print(f"== {title}")
    for f in layout.FIELDS:
        if f.kind in ("text", "music"):
            was = ""
            if compare is not None and show_text(f, compare) != show_text(f, payload):
                was = f"   (was {show_text(f, compare)!r})"
            print(f"  {f.key:28s} {show_text(f, payload)!r}{was}")
            continue
        raw = f.read(payload)
        mark = ""
        if compare is not None and f.read(compare) != raw:
            mark = f"   (was {f.label(f.read(compare))})"
        flag = "" if f.verified else "  [unverified]"
        print(f"  {f.key:28s} {f.label(raw):24s} raw={raw}{flag}{mark}")


def cmd_read(args: argparse.Namespace) -> None:
    session, ser = open_session(args.port)
    try:
        for esc in args.esc:
            s = session.read(esc)
            print_settings(s.payload, f"ESC {esc}  frames good={s.good_frames} bad={s.bad_frames}")
    finally:
        session.close()
        ser.close()


def cmd_backup(args: argparse.Namespace) -> None:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H%M%S")
    session, ser = open_session(args.port)
    try:
        for esc in args.esc:
            s = session.read(esc)
            path = out / f"esc{esc}_{stamp}.json"
            path.write_text(json.dumps(snapshot_json(s), indent=1))
            print(path)
    finally:
        session.close()
        ser.close()


def cmd_show(args: argparse.Namespace) -> None:
    doc = json.loads(Path(args.file).read_text())
    print_settings(payload_from_json(doc), f"{args.file} (ESC {doc.get('esc')})")


def cmd_set(args: argparse.Namespace) -> None:
    doc = json.loads(Path(args.file).read_text())
    before = payload_from_json(doc)
    changes = {}
    after = before
    for item in args.changes:
        key, _, value = item.partition("=")
        if key not in layout.BY_KEY:
            raise SystemExit(f"unknown field {key!r}; see `fields`")
        if key == "name":
            after = layout.encode_name(after, value)
        elif key == "music_notes":
            after = layout.encode_music(after, value)
        else:
            changes[key] = layout.parse_value(key, value)
    after = layout.encode(after, changes)
    doc.update(
        base_payload_hex=doc.get("base_payload_hex", before.hex()),
        payload_hex=after.hex(),
        page_b64=base64.b64encode(cipher.encode_page(after)).decode(),
        settings=layout.decode(after),
        edited_from=args.file,
    )
    Path(args.out).write_text(json.dumps(doc, indent=1))
    print_settings(after, f"{args.out}", compare=before)


def cmd_write(args: argparse.Namespace) -> None:
    doc = json.loads(Path(args.file).read_text())
    payload = payload_from_json(doc)
    if doc.get("esc") != args.esc:
        raise SystemExit(
            f"{args.file} was read from ESC {doc.get('esc')}, not {args.esc}. Settings such as "
            "direction differ per ESC: back up this ESC and edit that file instead."
        )
    # An edit made with `set` must land on the settings it was made from. A plain backup is a
    # restore and overwrites whatever the ESC holds.
    base = bytes.fromhex(doc["base_payload_hex"]) if "base_payload_hex" in doc else None
    session, ser = open_session(args.port)
    try:
        before = session.read(args.esc)
        if before.payload == payload:
            print(f"ESC {args.esc} already holds these settings; nothing written")
            return
        print_settings(payload, f"writing ESC {args.esc}", compare=before.payload)
        if input("write these settings? [y/N] ").strip().lower() != "y":
            raise SystemExit("not written")
        stamp = time.strftime("%Y-%m-%d_%H%M%S")
        saved = Path(args.file).with_name(f"esc{args.esc}_before_write_{stamp}.json")
        saved.write_text(json.dumps(snapshot_json(before), indent=1))
        print(f"saved the ESC's current settings to {saved}")
        after = session.write(args.esc, payload, expect_current=base)
        print(f"ESC {args.esc}: written and verified, {after.good_frames} good frames counted")
    finally:
        session.close()
        ser.close()


def cmd_fields(_: argparse.Namespace) -> None:
    for f in layout.FIELDS:
        rng = f"{f.lo}..{f.hi}" if f.kind in ("int", "enum") else f.kind
        opts = ""
        if f.options and len(f.options) <= 8:
            opts = "  " + ", ".join(f"{k}={v}" for k, v in f.options.items())
        flag = "" if f.verified else " [unverified]"
        print(f"{f.offset:3d} {f.key:28s} {rng:10s}{opts}{flag}")


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default=DEFAULT_PORT)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("read")
    r.add_argument("--esc", type=int, nargs="+", default=[0, 1])
    b = sub.add_parser("backup")
    b.add_argument("out")
    b.add_argument("--esc", type=int, nargs="+", default=[0, 1])
    s = sub.add_parser("show")
    s.add_argument("file")
    e = sub.add_parser("set")
    e.add_argument("file")
    e.add_argument("changes", nargs="+", metavar="KEY=VALUE")
    e.add_argument("-o", "--out", required=True)
    w = sub.add_parser("write")
    w.add_argument("file")
    w.add_argument("--esc", type=int, required=True)
    sub.add_parser("fields")
    args = p.parse_args()
    {
        "read": cmd_read,
        "backup": cmd_backup,
        "show": cmd_show,
        "set": cmd_set,
        "write": cmd_write,
        "fields": cmd_fields,
    }[args.cmd](args)


if __name__ == "__main__":
    main()
