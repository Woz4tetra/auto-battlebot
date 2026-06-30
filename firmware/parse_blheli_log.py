#!/usr/bin/env python3
"""Parse BLHeliSuite32 .xlg binary log files into JSON Lines format.

Usage:
    python3 parse_blheli_log.py <input.xlg>
    python3 parse_blheli_log.py <input.xlg> -o <output.jsonl>
"""

import argparse
import json
import struct
import sys
from pathlib import Path


def _try_record(data: bytes, lpos: int, length: int):
    """Decode one length-prefixed record at lpos. Returns (slen, sstart, send, text) or None."""
    if lpos + 4 > length:
        return None
    slen = struct.unpack_from("<I", data, lpos)[0]
    if slen < 1 or slen > 100:
        return None
    sstart = lpos + 4
    send = sstart + slen
    if send > length:
        return None
    chunk = data[sstart:send]
    printable = sum(1 for b in chunk if 32 <= b < 127)
    nulls = chunk.count(0)
    if printable + nulls < len(chunk) * 0.85:
        return None
    if printable < 1:
        return None
    text = chunk.decode("latin-1").rstrip("\x00").strip()
    if len(text) < 1:
        return None
    return (slen, sstart, send, text)


def extract_strings(data: bytes) -> list[dict]:
    """Extract length-prefixed text records from xlg binary data.

    The xlg format embeds strings after 0xFF delimiters as:
        FF [00|80] [00|FF] FF <4-byte LE length> <string bytes>
    """
    entries = []
    pos = 0
    length = len(data)

    while pos < length - 8:
        if data[pos] != 0xFF:
            pos += 1
            continue

        best = None
        for skip in (1, 3, 4):
            best = _try_record(data, pos + skip, length)
            if best:
                break

        if best:
            slen, sstart, send, text = best
            entries.append({"offset": hex(sstart), "length": slen, "text": text})
            pos = send
        else:
            pos += 1

    return entries


LABEL_SUFFIXES = (": ", ":")
KV_LABELS = {
    "ESC =",
    "CMD =",
    "ADRESS =",
    "PARAM_LEN =",
    "PARAM =",
    "ACK =",
    "CRC =",
}


def _frame_record(text, offset, cumulative_ms):
    """Return a frame record if text is a hex frame line, else None."""
    if text.startswith("24 4D") or text.startswith("2F ") or text.startswith("2E "):
        return {
            "type": "frame",
            "hex": text,
            "offset": offset,
            "time_ms": cumulative_ms,
        }
    return None


def _kv_record(text, entries, i, n, cumulative_ms):
    """Return a field record if text matches a known KV label and has a value, else None."""
    for kl in KV_LABELS:
        if text == kl.rstrip() or text.startswith(kl):
            if i + 1 < n:
                return {
                    "type": "field",
                    "label": text.rstrip(": "),
                    "value": entries[i + 1]["text"],
                    "offset": entries[i]["offset"],
                    "time_ms": cumulative_ms,
                }
            return None
    return None


def _timing_record(entries, i, n, cumulative_ms, offset):
    """Parse a 'Time elapsed (ms)' pair. Returns (record, updated cumulative_ms)."""
    try:
        cumulative_ms += int(entries[i + 1]["text"])
    except ValueError:
        pass
    record = {
        "type": "timing",
        "elapsed_ms": entries[i + 1]["text"] if i + 1 < n else "?",
        "cumulative_ms": cumulative_ms,
        "offset": offset,
    }
    return record, cumulative_ms


def _colon_field(text, entries, i, n, cumulative_ms, offset):
    """Return a field record for a 'label:' + short value pair, else None."""
    if not (text.endswith(":") and i + 1 < n):
        return None
    nxt = entries[i + 1]["text"]
    if len(nxt) >= 60:
        return None
    return {
        "type": "field",
        "label": text.rstrip(":").strip(),
        "value": nxt,
        "offset": offset,
        "time_ms": cumulative_ms,
    }


def group_entries(entries: list[dict]) -> list[dict]:
    """Coalesce adjacent label+value entries into structured records.

    Tracks cumulative time from 'Time elapsed (ms)' entries to stamp each record.
    """
    output = []
    i = 0
    n = len(entries)
    cumulative_ms = 0

    while i < n:
        text = entries[i]["text"]
        offset = entries[i]["offset"]

        frame = _frame_record(text, offset, cumulative_ms)
        if frame is not None:
            output.append(frame)
            i += 1
            continue

        if text in ("$[", "]"):
            i += 1
            continue

        kv = _kv_record(text, entries, i, n, cumulative_ms)
        if kv is not None:
            output.append(kv)
            i += 2
            continue

        if text == "Time elapsed (ms):" and i + 1 < n:
            record, cumulative_ms = _timing_record(entries, i, n, cumulative_ms, offset)
            output.append(record)
            i += 2
            continue

        field = _colon_field(text, entries, i, n, cumulative_ms, offset)
        if field is not None:
            output.append(field)
            i += 2
            continue

        output.append({"type": "text", "text": text, "offset": offset, "time_ms": cumulative_ms})
        i += 1

    return output


def main():
    parser = argparse.ArgumentParser(description="Parse BLHeliSuite32 .xlg log files")
    parser.add_argument("input", type=Path, help="Path to .xlg file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output .jsonl file (default: stdout)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Output raw extracted strings without grouping",
    )
    args = parser.parse_args()

    data = args.input.read_bytes()
    entries = extract_strings(data)

    if not args.raw:
        entries = group_entries(entries)

    out = open(args.output, "w") if args.output else sys.stdout
    try:
        for entry in entries:
            out.write(json.dumps(entry, ensure_ascii=False) + "\n")
    finally:
        if args.output:
            out.close()
            print(f"Wrote {len(entries)} entries to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
