# Hardware CAD notes

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## Build environment

### `cage_pack/battery_clamp.py`

Line 27, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `cage_pack/camera_shield.py`

Line 36, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `cage_pack/cup_arm.py`

Line 48, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `cage_pack/display_bracket.py`

Line 28, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `cage_pack/enclosure.py`

Line 71, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `cage_pack/enclosure_lid.py`

Line 38, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `cage_pack/pcba_cage.py`

Line 54, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `mr_stabs_mk2_boot_sleeve/mr_stabs_mk2_boot_sleeve.py`

Line 44, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `zed_box_mini_mount/zed_cheese_plate_adapter.py`

Line 41, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

### `zed_box_mini_mount/zed_x_one_s_bracket.py`

Line 41, `import recipe`:

```text
build123d-part/scripts, on the path when run through scripts/run
```

## Onshape push conventions

### `cage_pack/cage_pack_assembly.py`

Line 434, `above out["sma_bulkheads"] = Compound([_sma_bulkhead(y, z) for y, z in cl.ANTENNAS.values()])`:

```text
RP-SMA bulkheads, one per antenna, then each antenna screwed onto its bulkhead and
folded up: the same part three times, so Onshape holds one antenna model.
```

Line 982, `above ONSHAPE_PARTS = onshape_catalog.PARTS`:

```text
Onshape (the onshape skill): what each component is fastened to, on a pair of coaxial hole
edges the two share, so it follows that part when a dimension changes. The tray is the
fixed root. Anything not listed is inserted fixed where it is modelled: the glass, the
cells and their holder, the charger, the cables and most plugs share no hole with what
they sit on.
Onshape names and part properties, and which parts get their own document.
```

### `cage_pack/onshape_catalog.py`

Docstring at line 1:

```text
Onshape names and part properties for the cage pack, keyed by assembly component or
fastener group (cage_pack_assembly.ONSHAPE_PARTS).

Each entry sets the Part Studio tab, part and (for a purchased part) document name, `title`,
upper case the Onshape way, and the part's properties: Part number (the manufacturer's
part number), Vendor (the manufacturer) and Description (what it is and where it was
bought). `home` is "own" for an off-the-shelf part, which lives in its own document, and
"main" for a part designed here. `bom: False` sets "Exclude from all BOMs" (the cage wall,
the PCBA's parts, which are priced with the board).

Components with congruent shapes (the two cups, every M3 insert) share one entry's title
and become one Part Studio with several instances.

Sources: manufacturer pages, Digi-Key and McMaster listings, and for Amazon items the
listing's brand (from search snippets, checked 2026-11-11). Where a maker gives no part
number the Amazon ASIN stands in, and the description says so.
```

Line 322, `above "cup_screws": _mc(`:

```text
Fasteners (the M3 x 6 under the ZED is Onshape Standard Content)
```

## Recipe.py plane conventions

### `cage_pack/cup_arm.py`

Docstring at line 82:

```text
Profile on Top extruded across the arm, then the screw holes from the Front plane
(sketch x = X, y = Z; normal -Y, so an extrude's start is minus the Y it begins at).
```

### `zed_box_mini_mount/zed_x_one_s_bracket.py`

Line 131, `above {"op": "sketch", "name": "Front mount inserts", "plane": "Front", "profiles": [`:

```text
Front plane: sketch x = +X, sketch y = +Z, normal -Y; the holes run -Y from the
cheek fronts at y = plate_y.
```

## Lint exceptions: N806 for textbook matrix symbols

### `cage_pack/dynamics.py`

Line 41, `above import json`:

```text
Matrix and physics symbols keep their textbook capitals (M, K, C, N, J).
```

### `cage_pack/two_sheet.py`

Line 20, `above from __future__ import annotations`:

```text
Matrix and physics symbols keep their textbook capitals (M, P, N).
```

## Print masses

### `cage_pack/print_mass.py`

Docstring at line 1:

```text
Printed-part masses from PrusaSlicer instead of solid volume x a guessed fill factor.

    ~/.local/share/build123d-part/venv/bin/python print_mass.py [part ...]

Slices each part's STL (out/<part>/<part>.stl, written by check.py) with petg_p1s.ini
through the PrusaSlicer flatpak and reads "filament used [g]" from the G-code. Run
check.py on a part first so its STL is current. Settings that change the result live in
petg_p1s.ini: 4 perimeters, 5 top and 4 bottom layers, 25 % gyroid.
```

## Recipe.py conventions

### `zed_box_mini_mount/zed_cheese_plate_adapter.py`

Line 115, `above {"op": "chamfer", "edges": [(0, -p.width / 2, 0)], "size": p.bed_chamfer},`:

```text
Elephant's foot relief: one bed edge, tangent propagation takes the whole outline.
```
