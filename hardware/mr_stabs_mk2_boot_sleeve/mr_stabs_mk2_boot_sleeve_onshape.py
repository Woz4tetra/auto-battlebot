"""Onshape push of the Mk2 BOOT/RST sleeve on its own (../onshape skill).

The sleeve's tilted chassis relief and its off-plane pocket and holes are outside the
feature recipe, so it goes up as a STEP-imported part.
"""

TITLE = "MR STABS MK2 BOOT SLEEVE"


def components() -> dict:
    return {"boot_sleeve": {"part": "mr_stabs_mk2_boot_sleeve.py", "title": "MR STABS MK2 BOOT SLEEVE"}}
