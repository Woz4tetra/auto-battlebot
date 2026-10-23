#!/usr/bin/env python3
"""Write the BLHeli_32 field table for the Mk2 diagnostics page's ESC editor.

    venv/bin/python firmware/blheli/export_layout.py

Rerun after any change to auto_battlebot/blheli32/layout.py; tests/python/test_blheli32.py fails
while the two differ.
"""

from pathlib import Path

from auto_battlebot.blheli32 import layout

OUT = Path(__file__).resolve().parents[1] / "mr_stabs_mk2/web/esc_layout.json"


def main() -> None:
    OUT.write_text(layout.to_json())
    print(OUT)


if __name__ == "__main__":
    main()
