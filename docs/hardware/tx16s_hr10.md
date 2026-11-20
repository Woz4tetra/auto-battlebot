# TX16S HR10 trainer-port parts

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## Bar scan

### `tx16s_hr10_handle_clamp/bar_scan.py`

Docstring at line 1:

```text
Cut cross-sections of the TX16S MK2 carry-handle bar out of the full-radio scan.

The bar's front (pilot side) bulges toward the middle, so its section changes along X.
This writes closed (y, z) loops at fixed X stations, radio frame (scan_patch.py in
../tx16s_hr10_mount), for the clamp to loft its bore through.

Each loop is the scan points within 0.2 mm of the station, binned by angle about the
section centroid (5 deg bins, median radius, then a 3-bin circular mean).

    ~/.local/share/build123d-part/venv/bin/python         hardware/tx16s_hr10_handle_clamp/bar_scan.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Handle clamp: bar scan station range and crop

### `tx16s_hr10_handle_clamp/bar_scan.py`

Line 22, `STATIONS = np.arange(58.0, 80.01, 2.0)`:

```text
straight run of the right end; the corner bends from x 80
bar lies behind the radio body (back face y ~ 22)
```

## Handle clamp: render_context usage

### `tx16s_hr10_handle_clamp/render_context.py`

Docstring at line 1:

```text
Render the HR10 handle clamp, receptacle and mated plug on the full scanned TX16S.

Writes context_*.png into the check_assembly.py output directory and adds them to its
report.json, so page_assembly.py embeds them with the other drawings. Run
check_assembly.py first.

    ~/.local/share/build123d-part/venv/bin/python         hardware/tx16s_hr10_handle_clamp/render_context.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Handle clamp

### `tx16s_hr10_handle_clamp/tx16s_hr10_handle_clamp.py`

Docstring at line 1:

```text
TX16S MK2 HR10 handle clamp: holds an HR10A-7R-4S receptacle on the carry-handle bar.

The fastener-only alternative to ../tx16s_hr10_mount (VHB on the shell). Two printed
halves clamp the straight right end of the carry-handle bar with two M3 screws into
heat-set inserts; nothing is glued and no radio screw is touched. A tower cantilevers off
the clamp's right end over the bar's corner and stands the receptacle there, axis
vertical, so the plug and its cable come in from above. The AUX1 pigtail leaves the
tower's underside through a front notch, straight onto the right handle arm.

Spec
- Process: FDM PLA, both halves printed upright (radio +Z up): the top half on its split
  face, the bottom half on its underside. The bore crown and the hex cavity's ceiling
  print as bridges. The tower's floor is a one-sided 16 mm overhang 9 mm off the bed
  (check.py fdm.overhang WARN): enable slicer support under the tower only.
- Frames: geometry is built in the radio frame (../tx16s_hr10_mount/scan_patch.py: +X
  pilot's right, +Y away from pilot, +Z up). Each half is then shifted only, so its print
  bed is z = 0 and its origin sits at the clamp's centre. Params.half picks the half.
- Handle: fixed (user, 2026-10-25). Bar sections from the scan (bar_scan.py,
  tx16s_handle_bar_sections.npz) at x 58..80. The front face recedes from y 47.8 to 51.9
  across the clamp, so the bore is a ruled loft through those sections, not a prism.
- Bore tools: 0.2 mm radial clearance on the scan sections. Each half follows the bar's
  surface out from its widest points, and never within 0.15 mm of its split face; nearer
  the split the bore walls stand straight at the section's full width, so neither half
  has an undercut lip and the bar cannot reach past them. The last 1 mm at the right
  end is the x = 76 section opening 0.3 mm more toward the end face: the bar narrows
  toward +X, and a tapered bore meets that end face at under 70 deg. A 0.4 mm entry
  chamfer runs round the bore at both end faces.
- verify_scan.py against the raw scan: plug 14.8 mm clear; the bottom half's bore sits
  up to 0.18 mm into scan bumps the smoothed sections miss, which closes the 0.8 split
  gap to about 0.6 when tightened.
- Placement: clamp spans x 61..77, short of the corner bend at x 80. The tower spans
  x 77..93 over the corner (scan peak z -21.1 there) with a 1.5 mm gap under it. The arm
  drops forward from y 50 at x 83..88. The module bay is ahead of and below the bar
  (y < 25).
- Clamping: split at z -29, the section's widest point. The halves stand 0.8 apart on the
  bar; tightening the screws flexes the ears onto it.
- Fasteners: 2 x M3 x 14 SHCS from the top, counterbored, into M3 heat-set inserts
  (Ø4.0 hole, 5.7 long, fasteners.md) in the bottom half, at y 44 (front) and y 70 (back).
- Receptacle: same captive-nut scheme as the shell mount. M8x0.5 nut in a 10.1 AF hex
  cavity open at the tower's underside; the receptacle threads down into it and its key
  tab turns in a Ø10.6 x 0.8 counterbore under the flange.

Assembly
1. Press the two inserts into the bottom half with an iron.
2. Push the nut into the tower's hex cavity from below. Thread the receptacle in from
   the top until the flange seats, medium threadlocker on the thread.
3. Solder the pigtail through the cavity and lead it out of the front notch.
4. Put both halves on the bar, screws in, snug. Stop when the clamp no longer turns on
   the bar; the ears flex.

Open questions
- Grip: PLA on the bar's plastic with a scan-fit bore. If the clamp turns under a plug
  pull, add 0.5 mm silicone tape in the bore and reprint with gap 0.5.
- Key tab length and nut thickness are catalog gaps; same caveats as ../tx16s_hr10_mount.
```

## Handle clamp dimensions

### `tx16s_hr10_handle_clamp/tx16s_hr10_handle_clamp.py`

Line 89, `half: str = "top"`:

```text
"top" or "bottom"
Clamp, radio frame
clamp left end
clamp right end, short of the corner bend at x 80
widest point of the section
between the halves on the bar
constant-section bore at the right end
radial, about the scan's own accuracy; the 0.8 gap still closes
the lead widens by this much more over its run to the end face
bore walls stand at full section width this far into each half
entry chamfer at both end faces
2.25 wall outside the front counterbore
2.25 wall outside the back counterbore
4.3 over the bar's crown (z -21.3)
3.8 under the bar's lowest point at x 61 (z -33.7)
```

Line 105, `above hole_d: float = 8.3`:

```text
Tower, radio frame
over the bar's corner; arm at x 83..88
over the bar's crown (y 59.5)
hex flat 5.05 + 2.95 wall; tower meets the clamp at x 77
hex corner 5.83 + 2.0 wall
over the corner's scan peak (z -21.1)
scan, max z under the tower footprint
Receptacle, HR10 catalog (see ../tx16s_hr10_mount)
catalog max, for a 1.2 web under the counterbore
```

## Handle clamp bore

### `tx16s_hr10_handle_clamp/tx16s_hr10_handle_clamp.py`

Docstring at line 145:

```text
The handle bar from x 58 to 80, radio frame, lofted through the scan sections.

Ruled like the bore tools, so the two differ only by bore_clear; a smooth loft bulges
up to 0.2 mm past the ruled one between stations.
```

Line 217, `above last = stations[-1][1]`:

```text
The lead widens toward the end: the bar's x 78 section pokes 0.1 past the x 76 one
in places, and a bore that widens toward an end face meets it at an obtuse angle.
```

## Handle clamp: raw-scan verification

### `tx16s_hr10_handle_clamp/verify_scan.py`

Docstring at line 1:

```text
Check both clamp halves and a mated plug against the raw TX16S scan.

check.py and check_assembly.py measure against the lofted bar from x 58 to 80. The bar's
corner under the tower, the right arm, and everything the plug passes are only in the
scan, so this measures every scan vertex near the clamp.

    ~/.local/share/build123d-part/venv/bin/python         hardware/tx16s_hr10_handle_clamp/verify_scan.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Shell mount: render_context usage

### `tx16s_hr10_mount/render_context.py`

Docstring at line 1:

```text
Render the HR10 mount, receptacle and mated plug on the full scanned TX16S.

Writes context_*.png into the check.py output directory and adds them to its report.json,
so page.py embeds them with the other renders. Run check.py first.

    ~/.local/share/build123d-part/venv/bin/python hardware/tx16s_hr10_mount/render_context.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Scan and radio frame

### `tx16s_hr10_mount/scan_patch.py`

Docstring at line 1:

```text
Cut the TX16S MK2 top-right height map out of the full-radio scan.

The scan is a 3MF (mm) of the whole radio, 3.0M vertices, posed upside down and rolled in
the scanner frame. This rotates it into the radio frame the mount uses and writes the top
surface over the right shoulder block as a 0.5 mm height map.

Radio frame: +X the pilot's right, +Y away from the pilot (the front face normal is -Y),
+Z up. Origin is the scanner's, so only directions mean anything.

Frame fit, done once on the full scan:
- Front face normal: area-weighted mean of face normals within 4 deg of the screen plane.
- Up: the four side walls cluster at 73.5 / 163.5 / 253.5 / 343.5 deg in the scanner YZ
  plane; 253.5 deg points out of the top (the handle side).

    venv/bin/python hardware/tx16s_hr10_mount/scan_patch.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

Line 27, `FRONT_NORMAL = np.array([0.99840818, 0.05007949, -0.02594522])`:

```text
scanner frame
side-wall cluster pointing out of the top, scanner YZ plane
Top-right block window, radio frame (mm). Covers the mount footprint, the two outer
toggles, the handle hinge and the back edge drop.
```

## Shell mount

### `tx16s_hr10_mount/tx16s_hr10_mount.py`

Docstring at line 1:

```text
TX16S MK2 HR10 trainer-port mount: holds an HR10A-7R-4S receptacle on top of the radio.

The Jetson trainer link runs on the radio's AUX1 UART (bottom front face). A pigtail from
AUX1 runs up the outside of the shell to this block, which stands the receptacle on top of
the right shoulder block, tilted back so the mating plug and its cable lead away from the
pilot. The block is bonded with VHB tape to a contour taken from a 3D scan of the radio:
the right shoulder block has no screws to borrow, and the module bay, which may change,
is the only other flat area on top.

Spec
- Process: FDM PLA, panel face on the bed, contoured base up. No supports.
- Frame: origin at the receptacle's panel-hole centre on the panel face; +Z runs into the
  block toward the radio (print direction). Radio frame (scan_patch.py: +X pilot's right,
  +Y away from pilot, +Z up) maps here by radio_to_part().
- Placement: receptacle axis meets the shell at radio (95.3, -10.5), tilted 20 deg back
  about radio X. Footprint stays in x 87.8..102.8: ahead of the carry-handle hinge arm
  (x 81..88, only behind y 12) and inboard of the two outer shoulder toggles. Toggle A's
  boss flank starts at x 102 for y -2..6, so the back-right corner tapers in. The front
  wall is vertical at radio y = -24, ahead of the dome's front roll-off. The module bay
  (x < 70) is 18 mm away. verify_scan.py checks all of this against the raw scan.
- Interfaces:
  - Shell: base is the scan height map (tx16s_top_right_heightmap.npz) offset 1.1 mm
    along its normal for 3M VHB 5952 (1.1 mm, made for painted and low-energy plastics).
  - HR10A-7R-4S(73), Hirose HR10 catalog p.7 and p.20: panel hole 8.1 +0.1, key slot
    1.6 +0.1 to 5.1 from centre, panel 0.7 to 2.0 thick, M8x0.5 nut 10 AF.
  - Captive nut: the M8x0.5 nut sits in a 10.1 AF hex cavity under the panel, open to
    the base, and the receptacle threads down into it. A nut under a keyed panel needs a
    10 mm socket (Ø15) or pliers to turn it, and neither fits a block this narrow. The
    key tab turns with the receptacle, so it rides in a Ø10.6 x 0.8 counterbore under
    the flange instead of a slot (tab length <= 0.7, the catalog's thinnest panel).
  - Hole Ø8.3: catalog 8.1 +0.1, +0.2 FDM undersize (fdm.md).
  - Wire exit: a groove the full width of the hex flat (5.83) from the cavity out the
    back wall. Flat roof, at least 4.5 mm over the shell along its run.
- Critical dimensions: panel 2.0 thick (1.2 web under the counterbore); hole Ø8.3;
  counterbore Ø10.6 x 0.8; hex cavity 10.1 AF, 11 deep from the tape face at the axis,
  a flat toward the groove; walls 1.67 at the hex corners (+-X); 0.5 chamfer on every
  base edge.
- Edge geometry: the walls run along the tilted axis and the shell slopes away under
  them, so every base edge, groove edge and corner here was placed to keep check.py's
  1.2 mm minimum wall. The chamfers are fragile: base_chamfer 0.45 or 0.55, or
  taper_u 4.5, fails to build.

Assembly
1. Solder nothing yet. Push the nut into the hex cavity from the base and hold it up
   against the panel with a pencil.
2. Thread the receptacle in from the top by its body until the flange seats. A drop of
   medium threadlocker on the thread.
3. Solder the pigtail to the cups through the base, lay it in the groove.
4. Clean the shell with IPA, VHB 5952 on the base, press 30 s, wait 24 h before loading.

Open questions
- Handle swing: the scan shows the handle up and back. If it folds forward over the top,
  check the plug clears its arm (verify_scan.py reports the gap to the scanned arm only).
- Key tab length and the flange's bearing ring: the counterbore leaves the flange 0.2 mm
  of PLA to sit on (r 5.3 to 5.5). Print to 3 mm height first and seat a receptacle; if
  the tab is longer than 0.8, raise key_depth (and panel_t cannot follow past 2.0).
- Receptacle flange thickness and nut thickness are not in the catalog; the mate models
  them as 1.5 and 2.0.
- AUX1 connector type and pin order: meter the port before wiring the pigtail.
```

Line 252, `above trim_dir = (to_part * Pos(0, -1, 0)).position - (to_part * Pos(0, 0, 0)).position`:

```text
Round the trim's two side edges: square ones meet the tilted base in a sharp
three-way corner.
```

Line 265, `above hexagon = RegularPolygon(p.nut_af / 2, 6, major_radius=False)`:

```text
Captive-nut cavity under the panel, open through the base. A flat faces the wire
groove (part -Y): a groove leaving through a hex corner leaves knife edges.
```

Line 273, `above o = axis_origin(p)`:

```text
Wire groove from the cavity out the back wall. The roof is flat in the part frame,
square to the hex flat it leaves through (a roof following the shell meets that flat
at an acute angle), and sits at the lowest roof that keeps groove_h over the shell
along the whole run. The groove is as wide as the hex flat, so its walls carry on
from the neighbouring hex faces with no short edges between.
```

Line 286, `above base = block.faces().filter_by(GeomType.BSPLINE).sort_by(lambda f: f.area)[-1]`:

```text
Edge break around the contoured base. The walls run along the tilted axis and the
shell slopes away under them, so the base meets them at down to 55 deg: a feather
edge that tears off the tape. Chamfer every edge of the base face, cavity rim included.
The base is the largest B-spline face; the trim fillets come out as B-splines too.
```

## Shell mount dimensions

### `tx16s_hr10_mount/tx16s_hr10_mount.py`

Line 96, `above r_corner: float = 3.0`:

```text
Placement, radio frame (scan_patch.py)
between hinge arm and toggle A's boss (from x 102)
nearer toggle B (y -20, flank x 106) than A (y +2, flank x 102)
plug leans back, away from the pilot
vertical front wall, ahead of the dome roll-off at y -26
Footprint across the axis: u along radio X, v toward the back
hex 11.66 across corners + 1.67 walls
front trim crosses the bed here, so the bed edge stays square
back edge, short of the shell's back drop at y = 11
back-right taper starts here...
...and ends here at v_back: toggle A's boss flank reaches x 102
```

Line 108, `r_taper_corner: float = 0.6`:

```text
back-right: arc starts outboard of the groove wall (x 2.92)
front trim's side edges; 2.0 leaves the base chamfer no room
Bond
3M VHB 5952 thickness
breaks the acute edges where the tilted base meets the walls
Receptacle, HR10 catalog
catalog 0.7 to 2.0; the max, for a 1.2 web under the counterbore
8.1 +0.1 catalog, +0.2 FDM undersize (fdm.md)
key tab tip r 5.1, +0.1 catalog, +0.1 FDM
tab <= 0.7, the catalog's thinnest panel
10 AF nut, snug sliding fit (fdm.md)
tape face to panel: 7.7 behind the panel + wire bend
Wire exit
least clear height over the shell along the groove; 3.5 OD cable
elephant's foot on the bed edge, fdm.md
```

Line 310, `above key = Pos(0, -4.45, 0) * Box(1.5, 1.1, 0.7, align=C_MIN)`:

```text
Key tab: r 3.9 to 5.0, 1.5 wide, 0.7 long; it ends up wherever the thread stops.
```

## Shell mount surface fit

### `tx16s_hr10_mount/tx16s_hr10_mount.py`

Line 179, `above SURFACE_X = (86.5, 107.0)`:

```text
Height map region the base follows. Outside it the slab continues flat from its edge, so
the tilted prism always ends inside the slab; a B-spline through the real back-edge drop
and toggle bosses overshoots by centimetres. Inside, the spline is within 0.3 mm of the
map except the y = -27.5 and 8.5 edge rows (0.5 to 0.9 mm above it: a gap, never a clash).
```

## Printing

### `tx16s_hr10_mount/tx16s_hr10_mount.py`

Line 246, `above front = Pos(p.axis_x, p.front_y, -60) * Box(`:

```text
Vertical front wall in the radio frame. It leans 20 deg from the print axis, toward
the part, so it prints without support.
```

## Shell mount: raw-scan verification

### `tx16s_hr10_mount/verify_scan.py`

Docstring at line 1:

```text
Check the HR10 mount and a mated plug against the raw TX16S scan, not the height map.

check.py measures against a smoothed slab of the top surface, which cannot hold the
handle arm, the toggle levers, or anything the plug passes. This places the block and the
plug envelope in the radio frame and measures every scan vertex near them.

    ~/.local/share/build123d-part/venv/bin/python hardware/tx16s_hr10_mount/verify_scan.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Rod clamp: render_context usage

### `tx16s_hr10_rod_clamp/render_context.py`

Docstring at line 1:

```text
Render the HR10 rod clamp, receptacle and mated plug on the full scanned TX16S.

Writes context_*.png into the check_assembly.py output directory and adds them to its
report.json, so page_assembly.py embeds them with the other drawings. Run
check_assembly.py first.

    ~/.local/share/build123d-part/venv/bin/python         hardware/tx16s_hr10_rod_clamp/render_context.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Rod clamp

### `tx16s_hr10_rod_clamp/tx16s_hr10_rod_clamp.py`

Docstring at line 1:

```text
TX16S MK2 HR10 rod clamp: holds an HR10A-7R-4S receptacle on the bare handle rod.

Variant of ../tx16s_hr10_handle_clamp for the handle with its plastic sheath removed. Two
printed halves close on the bare Ø5 rod around the handle's right corner, so they grip the
end of the back run and the top of the right arm at once. The two legs point in different
directions, so the clamp cannot turn about either one: it is located by the bend, and the
screws only have to hold it closed. A tower on the top half stands the receptacle with its
axis vertical, as on the other two mounts.

Spec
- Process: FDM PLA. Top half prints on its split face, bottom half on its underside; both
  are built in the rod plane, so neither needs flipping.
- Rod frame (u, v, n): origin on the arm axis at the corner, u along the run (pilot's
  left, radio -X), v down the arm toward the pilot, n normal to the rod plane (up, tilted
  20.6 deg toward the pilot). Radio frame as ../tx16s_hr10_mount/scan_patch.py.
- Rod: Ø5.00, cylinder fit to the bare arm in the scan (4087 vertices, 0.11 RMS, y 32..42).
  The fitted arm axis, run up to the back run, meets the run sheath's centre height within
  0.15 mm, so the rod bends in one plane (n = 0).
- Bore: Ø5.2 (FDM close fit, dfm/fdm.md: holes print 0.1-0.2 under) with the halves 0.8
  apart, so the screws close each half 0.1 onto the rod and still leave 0.6 of squeeze.
- Run position: unknown inside the old sheath. Centred it would sit at v +0.2; flush with
  the sheath's back face, as the arm sat flush with its outer face, at v -3.1. The run's
  groove is a slot from v -3.4 to +0.5, tight in n only.
- Bend radius: unknown, it was inside the sheath. The corner is a pocket clear of the rod
  for any centreline radius up to 10 mm; the grips start 10 mm from the corner on each leg.
  check.py passes mate.rod with the rod at both ends of the slot and bend radii 2.6 and 10
  (--params '{"rod_v": -3.4, "rod_bend_r": 2.6}' and so on).
- Fasteners: 3 x M3 x 8 SHCS along -n, counterbored in the top half, into M3 hex nuts in
  pockets on the bottom half's underside (fasteners.md). One beside the run, one beside the
  arm, one outside the corner, so each groove has a screw on either side of it.
- Receptacle: M8x0.5 nut in a 10.1 AF hex cavity under a 2.0 panel (catalog max), Ø8.3
  hole with a Ø10.6 x 0.8 key-tab counterbore, as the other mounts. The cavity opens to
  pilot's right (+X) as a slot the nut slides in through; the pigtail leaves by the same
  slot and drops down outside the arm.

Assembly
1. Slide the M8 nut into the tower's side slot, thread the receptacle down into it from
   the top until the flange seats, medium threadlocker on the thread.
2. Solder the pigtail and lead it out of the slot.
3. Drop the three M3 nuts into the bottom half's pockets, close both halves on the corner,
   screw up evenly until the halves stop rocking on the rod.

Open questions
- Run position and bend radius (above). If the halves bottom out on the rod, check the
  run sits inside the slot; if the clamp slides along the arm, shim the slot.
```

## Rod clamp scan fit

### `tx16s_hr10_rod_clamp/tx16s_hr10_rod_clamp.py`

Line 77, `above ARM_POINT = np.array([85.552, 37.0, -34.645])`:

```text
Cylinder fit to the bare arm (see docstring): a point on its axis and its direction.
```

## Rod clamp dimensions

### `tx16s_hr10_rod_clamp/tx16s_hr10_rod_clamp.py`

Line 85, `half: str = "top"`:

```text
"top" or "bottom"
scan fit
rod + 0.2, FDM close fit
between halves on the rod
each half's outer face from the rod plane: 3.4 over the bore
Run slot: rod centre anywhere from flush-back to centred (docstring)
```

Line 93, `bend_max: float = 10.0`:

```text
corner pocket clears centreline radii up to this
grip on the run ends at u 27 (radio x 59)
grip on the arm ends at v 26 (radio y 33, clear of the body)
outside the bore, in plane
Screws, rod frame (u, v)
```

Line 117, `above rod_v: float = -1.45`:

```text
Rod shown in the mates, rod frame (unknowns: any value inside the ranges above)
```

## Rod clamp checks

### `tx16s_hr10_rod_clamp/verify_scan.py`

Docstring at line 1:

```text
Check the rod clamp halves, the plug and a hex driver against the raw TX16S scan.

The scan still has the handle sheath on, so vertices inside the sheath's envelope (rod
frame: the run within v -6..10.5, the arm above v 16, both within 5.6 of the rod plane)
are dropped; everything else (the bare arm, the radio body) must stay outside the parts.
The driver is a 2.5 hex bit (Ø2.5) in each counterbore and a Ø7 x 80 shaft up the screw
axis from the top face, which must miss the tower and the radio.

    ~/.local/share/build123d-part/venv/bin/python         hardware/tx16s_hr10_rod_clamp/verify_scan.py         ~/Downloads/radiomaster-tx16-1.snapshot.2.zip
```

## Workarounds

- `tx16s_hr10_handle_clamp/tx16s_hr10_handle_clamp.py`: the wire steps out to full width at the
  chain's ends. The step is horizontal when the chain is clipped at `z_cut`, and a short collinear
  piece of wall when the chain ends on the widest point. The 0.05 mm `step` nudge keeps the step
  edge non-degenerate when unclipped. Unverified without it.
- `tx16s_hr10_mount/tx16s_hr10_mount.py`: the back-right corner gets the tight `r_taper_corner`
  so its arc starts outboard of the wire groove's wall (x 2.92). A wall that clips the arc leaves a
  knife edge.
- `tx16s_hr10_rod_clamp/tx16s_hr10_rod_clamp.py:rod`: the straight legs are plain cylinders so the
  hole measurement reads them. Only the bend is swept.
