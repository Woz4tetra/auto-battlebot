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
            lpos = pos + skip
            if lpos + 4 > length:
                continue
            slen = struct.unpack_from("<I", data, lpos)[0]
            if slen < 1 or slen > 100:
                continue
            sstart = lpos + 4
            send = sstart + slen
            if send > length:
                continue
            chunk = data[sstart:send]
            printable = sum(1 for b in chunk if 32 <= b < 127)
            nulls = chunk.count(0)
            if printable + nulls < len(chunk) * 0.85:
                continue
            if printable < 1:
                continue
            text = chunk.decode("latin-1").rstrip("\x00").strip()
            if len(text) < 1:
                continue
            best = (slen, sstart, send, text)
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
    "ESC =", "CMD =", "ADRESS =", "PARAM_LEN =", "PARAM =",
    "ACK =", "CRC =",
}


def group_entries(entries: list[dict]) -> list[dict]:
    """Coalesce adjacent label+value entries into structured records."""
    output = []
    i = 0
    n = len(entries)

    while i < n:
        text = entries[i]["text"]
        offset = entries[i]["offset"]

        if text.startswith("24 4D") or text.startswith("2F ") or text.startswith("2E "):
            output.append({"type": "frame", "hex": text, "offset": offset})
            i += 1
            continue

        if text in ("$[", "]"):
            i += 1
            continue

        matched_kv = False
        for kl in KV_LABELS:
            if text == kl.rstrip() or text.startswith(kl):
                if i + 1 < n:
                    output.append({
                        "type": "field",
                        "label": text.rstrip(": "),
                        "value": entries[i + 1]["text"],
                        "offset": offset,
                    })
                    i += 2
                    matched_kv = True
                break
        if matched_kv:
            continue

        if text.endswith(":") and i + 1 < n:
            nxt = entries[i + 1]["text"]
            if len(nxt) < 60:
                output.append({
                    "type": "field",
                    "label": text.rstrip(":").strip(),
                    "value": nxt,
                    "offset": offset,
                })
                i += 2
                continue

        output.append({"type": "text", "text": text, "offset": offset})
        i += 1

    return output


def main():
    parser = argparse.ArgumentParser(description="Parse BLHeliSuite32 .xlg log files")
    parser.add_argument("input", type=Path, help="Path to .xlg file")
    parser.add_argument(
        "-o", "--output", type=Path, default=None,
        help="Output .jsonl file (default: stdout)",
    )
    parser.add_argument(
        "--raw", action="store_true",
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
