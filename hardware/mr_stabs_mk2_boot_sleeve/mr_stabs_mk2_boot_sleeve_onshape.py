"""Onshape push of the Mk2 BOOT/RST sleeve on its own (build123d-part onshape skill).

The sleeve defines features(), so it goes up as 10 native Part Studio features.
"""

TITLE = "MR STABS MK2 BOOT SLEEVE"


def components() -> dict:
    return {
        "boot_sleeve": {"part": "mr_stabs_mk2_boot_sleeve.py", "title": "MR STABS MK2 BOOT SLEEVE"}
    }
