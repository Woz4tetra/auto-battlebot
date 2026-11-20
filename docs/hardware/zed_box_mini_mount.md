# ZED Box Mini mount

These notes were moved out of source comments and docstrings in `hardware/` at commit
`23e253c1`. Names, line numbers and numbers are as they were in the code then.

## Assembly page

### `zed_box_mini_mount/assembly_page.py`

Docstring at line 1:

```text
Build the assembly review page for the ZED Box Mini cheese plate adapter.

Run in the build123d-part skill venv from this directory:

    ~/.local/share/build123d-part/venv/bin/python assembly_page.py path/to/zed_box_mini.step

The STEP is Stereolabs' "ZED Box Mini Wifi with fan.step" from stereolabs.com/3dmodels. It is
33 MB, so it stays out of git. Writes out/assembly/: index.html (from assembly_page.html with
the measured data injected) and one mesh JSON per part group for the 3D viewer, and
out/step/: the adapter, cheese plate, and ZED Box Mini as STEP in the assembly frame.
```

## Vendor STEP frames

### `zed_box_mini_mount/assembly_page.py`

Line 45, `above ZED_BASE_Y = 3.5`:

```text
STEP frame: X across the box, Y up (base sheet bottom at Y = 3.5), Z along the box (0..143).
Adapter frame: X along the box, Y across, Z up, base sheet bottom on the adapter top face.
```

### `zed_box_mini_mount/zed_x_one_s_bracket.py`

Line 62, `above CAM_FRONT_Y = -76.2705954`:

```text
Camera STEP frame: lens toward -Y, front face at Y = -76.2706, front M2 square centered on
(X, Z) = (0.5142, -17.6945). Measured from the STEP; matches datasheet p.8 (25 x 25 body, 21 mm
square). Solid 9 of the STEP is the FOV frustum surface, not hardware.
```

## CP10 reference model

### `zed_box_mini_mount/innorel_cp10.py`

Docstring at line 1:

```text
INNOREL CP10 cheese plate (Amazon B0DRYFL595): 200 x 100 x 10 mm 6061-T6 plate.

A reference model for the adapter assembly, not a part to make. Hole positions were measured
from INNOREL's dimensioned drawing (Amazon image 71V4ad30vkL), scaled to the 200 x 100
outline at 6.13 px/mm, then made symmetric about both axes. After symmetrizing, the counts
match the drawing exactly: 106 x 1/4-20, 27 x 3/8-16, 44 locating holes.

Dimensions printed on the drawing, which the model uses directly: slot width 6.4, slot
counterbore 10.3, center slot pitch 24.7 / 74.3 (so center slots at x = ±49.5), center slot
rows 42.3 apart, end slots 27 long on centers 23.5 + 27 apart, edge 3/8-16 holes at 0 and
±76.7, edge hole pitch 9.

Coordinates: origin at the center of the top face, X along the 200 mm length, +Z up. Threaded
holes are modeled at their tap drill size (1/4-20: Ø5.1, 3/8-16: Ø8.4).

Open questions
- The slot counterbore depth is not on the drawing; INNOREL's store has the same drawing
  (Size-details-of-CP10.jpg) and no datasheet or CAD turned up. 6.5 mm comes from the close-up
  photo (Amazon image 71aXTwjp85L), where the counterbore wall is about twice the height of the
  through lip below it. Measure it with a caliper's depth rod.
- Edge hole depth (8.0) and corner radius (5.0) are not on the drawing either.
- Positions without a printed dimension carry about ±0.3 mm of image-scaling error.
```

Line 43, `TAP_1_4_20 = 5.1`:

```text
#7 drill, fasteners.md
5/16" drill
measured from the drawing
unlabeled on the drawing, measured
```

Line 89, `length: float = 200.0`:

```text
drawing
drawing
drawing
not on the drawing
drawing
drawing
estimated from photo 71aXTwjp85L, leaves a 3.5 mm lip
drawing: 74.3 / 2 + 24.7 / 2
drawing
drawing
measured
drawing: 23.5 / 2 + 27 / 2
drawing
not on the drawing
```

## Ball head reference model

### `zed_box_mini_mount/smallrig_2948_ball_head.py`

Docstring at line 1:

```text
SmallRig 2948 mini ball head (Amazon B09G9PFQCN, sold as the 2948B two-pack), cold shoe removed.

Reference model for the camera bracket assembly, not a part to make. SmallRig publishes no
drawing or CAD for it. What is known:

- Listing (smallrigreseller.com, 2948B): 49 x 33 x 25 mm overall with the cold shoe fitted,
  47 g, 1.5 kg load, 360° pan, 135° tilt.
- Top: male 1/4-20 stud on a round platform with a rubber pad. Bottom: female 1/4-20 once
  the cold shoe adapter is unscrewed, which is how it mounts here.
- Everything else is scaled from SmallRig's side-view product photo (2948b_4_.jpg, about
  24 px/mm, using the listed 25 mm base width): body Ø19 at the top flaring to Ø23 at the
  bottom, 27.5 mm tall; a drop notch on one side lets the neck fall to 90°; the wing knob is
  on the opposite side; the platform is about Ø26 x 5.

Coordinates: origin at the center of the bottom face, which seats on the mounting surface;
+Z up the body axis. The drop notch is on +X, the knob on -X.

Pose: `pan` turns the whole head about Z (it is free to spin on its stud). `tilt` swings the
neck about the ball center toward `tilt_dir` (degrees in the body frame, 0 = into the notch).
Into the notch it reaches 90°; any other direction stops at about 20°.
`top_location(p)` gives the platform's top-face center with +Z along the stud, which is
where a mounted part's bottom face sits.

Open questions
- Every dimension except the overall 49 x 33 x 25 is photo-scaled, about ±1 mm.
- Bottom thread depth is not published; 8 mm assumed. Measure it before choosing a stud
  length: the 1/2" set screw here puts 6.35 mm into it.
```

Line 53, `body_h: float = 27.5`:

```text
photo scale
photo scale
photo scale, listing gives 25 across the cold shoe
ball center above the bottom face, photo scale
photo scale
photo scale
ball center to platform underside, photo scale
photo scale, octagon across corners
photo scale, metal plus rubber pad
photo scale; 1/4-20 male
1/4-20 major diameter
1/4-20 tap drill, fasteners.md
assumed
neck + clearance
photo scale
from axis; 33 mm listed depth minus half the 23 mm body
```

## Cheese plate adapter

### `zed_box_mini_mount/zed_cheese_plate_adapter.py`

Docstring at line 1:

```text
ZED Box Mini to cheese plate adapter: a printed plate sandwiched between the ZED Box Mini
and an INNOREL CP10 cheese plate (200 x 100 x 10 mm, 6061).

Spec
- Process and orientation: FDM PLA, 0.4 mm nozzle. Bottom face (cheese plate side) on the
  bed, so the counterbores and insert holes open upward and nothing overhangs.
- Coordinates: origin at the center of the bottom face. X runs along the cheese plate's
  200 mm length and the ZED's long axis, Y along the 100 mm width, +Z up toward the ZED.
- ZED Box Mini: the bottom is a 1 mm sheet with two side ears (104 mm across, 60 mm long);
  each ear has 2 x Ø3.6 holes. Measured from Stereolabs' STEP ("ZED Box Mini Wifi with
  fan.step", stereolabs.com/3dmodels): holes at (±24.0, ±47.3), a 48 x 94.55 mm pattern. The
  product page says "92 x 45 mm", which does not match the CAD. Body 143 x 86 mm.
- ZED to adapter: 4 x M3 x 6 mm heat-set inserts, Ø5 knurl, from the shop's stock ("M3x6x5").
  They sit in Ø4.2 through holes, flush with the top. Screws: M3 x 4 to M3 x 6 down through
  the ears. An M3 x 6 ends 1.7 mm above the bottom face; an M3 x 8 would stick out 0.3 mm
  and jack the adapter off the cheese plate.
- Adapter to cheese plate: 4 x 1/4-20 x 1/2" flanged button head (80/20 3342, the screw in
  McMaster 47065T142 without its T-nut), in counterbores under the ZED body. Install them before the ZED: the ZED's bottom sheet covers the heads.
  The 2.7 mm floor under the head puts the screw tip flush with the cheese plate's bottom
  face, so all 10 mm of the plate's thread is engaged.
- Cheese plate hole positions come from INNOREL's dimensioned Amazon image
  (m.media-amazon.com/images/I/71V4ad30vkL), scaled to the 200 x 100 outline at 6.13 px/mm.
  X = ±49.5 matches the drawing's own slot dimensions (74.3 / 2 + 24.7 / 2); Y = ±30.85 is
  scaled from the image only.

Open questions
- Measure the cheese plate's 1/4-20 Y spacing (61.7 mm here) before printing. The clearance
  holes are Ø7.0 so ±0.3 mm of image error still fits, and `plate_hole_y` is a Param.
- The flange size is unverified: 80/20's drawing would not load. The counterbore assumes
  a flange up to Ø0.60" (15.2 mm) and a head up to 0.15" (3.8 mm). Measure one; if it is
  bigger, raise `cbore_d` / `cbore_depth` and `thickness` together to keep the 2.7 mm floor.
- Insert hole Ø4.2 is a guess for a generic Ø5 knurl insert. Print a test hole if there is
  time; go to 4.0 if the insert drops in loose.
- 1/4-20 engagement is the plate's full 10 mm (1.57 D), over the 1.5 D rule for aluminum.
  The 2.7 mm of PLA under each flange is the weak link.
```

Line 63, `length: float = 143.0`:

```text
X: matches the ZED body length (STEP)
Y: matches the ZED ears (STEP); overhangs the 100 mm plate 1.9/side
Z: counterbore + 2.7 floor; the screw tip lands flush under the plate
vertical corners
elephant's foot relief, dfm/fdm.md
```

## Dimension sources

### `zed_box_mini_mount/zed_cheese_plate_adapter.py`

Line 50, `above @dataclass`:

```text
Mating hardware, used only by mates().
INNOREL CP10
ZED Box Mini STEP, main heatsink
bottom sheet and ears, STEP
ear span along X, total width across ears, STEP
STEP
80/20 3342, unverified
80/20 3342
fasteners.md; longest that fits
```

Line 69, `above insert_len: float = 6.0`:

```text
ZED Box Mini ear holes, STEP
±, along the ZED's long axis (48 mm pitch)
±, across (94.55 mm pitch in the STEP, rounded)
Shop-stock M3 x 6 x Ø5 heat-set insert
generic Ø5 knurl; print a test hole
```

Line 77, `above def features(p: Params) -> list[dict]:`:

```text
INNOREL CP10 1/4-20 holes used, from the plate drawing
±, drawing: 74.3 / 2 + 24.7 / 2
±, scaled from the drawing image; measure
1/4-20 x 1/2 flanged button head, 80/20 3342
6.75 free clearance (fasteners.md) + 0.25 FDM undersize
flange up to 15.2 (unverified) + 0.8 clearance and FDM undersize
head up to 3.8 (unverified) + 0.2 below the ZED sheet
```

## Cheese plate adapter: feature plan

### `zed_box_mini_mount/zed_cheese_plate_adapter.py`

Docstring at line 87:

```text
The part as 4 CAD features (recipe.py), replayed by build() and by the onshape skill.

One sketch holds every profile. The plate is the outline region extruded full height,
which leaves the insert holes and Ø16 counterbores through. The counterbore floors are
the Ø7-Ø16 rings added back up to 2.7 mm, so no feature needs a sketch on the top face.
```

Line 101, `above + [{"circle": (x, y, p.insert_hole)} for x, y in holes]`:

```text
M3 heat-set insert holes for the ZED ears, through so the screw length is free.
```

## Cheese plate adapter: modelling conventions

### `zed_box_mini_mount/zed_cheese_plate_adapter.py`

Line 155, `above ins = (`:

```text
Installed insert: brass melts into the hole wall, so it overlaps the part.
```

## Camera mount options

### `zed_box_mini_mount/zed_x_one_s_assembly.py`

Docstring at line 1:

```text
ZED Box Mini on the INNOREL CP10 cheese plate, with the ZED X One S on a ball head.

Extends the ZED Box Mini stack (zed_cheese_plate_adapter.py, assembly_page.py) with the
camera: ZED X One S -> Stereolabs front mount (ACC-151300) -> printed bracket
(zed_x_one_s_bracket.py) -> SmallRig 2948 ball head -> any 1/4-20 hole on the cheese plate.

The ball head takes a 1/4-20 x 1/2" set screw (McMaster 92311A537) as a double-ended stud:
6.35 mm into the plate's tapped hole, 6.35 mm into the head's bottom thread. Every option
below uses a hole the adapter and ZED Box Mini leave open. Pick one with MOUNT_OPTION:

| Option | Hole | Head | Camera |
| --- | --- | --- | --- |
| top_end (default) | top face, (-88.25, 0) | upright, 15° tilt | upright, looking -X, 15° down |
| underside | bottom face, (88.25, 0) | hanging, 30° into the notch | inverted, +X, 30° down |
| long_edge | long edge, x = 90, tapped 8 mm deep | 80° into the notch | upright, +Y, 10° down |
| short_edge | short edge, y = 13.95, 8 mm deep | 70° into the notch | upright, +X, 20° down |

The underside option images upside down; set the camera's flip in the ZED SDK.

top_end sits at the -X end: on the +X hole the head's body runs into the ZED Box Mini's
GMSL jacks, which stick 9.75 mm out of that end (check_assembly catches it by sampling).

    MOUNT_OPTION=long_edge scripts/run check_assembly.py zed_x_one_s_assembly.py         --out out/assembly_long_edge

Coordinates: the adapter frame, the same as assembly_page.py. Origin at the center of the
cheese plate's top face, X along its 200 mm length, +Z up toward the ZED Box Mini.

The ZED Box Mini comes from out/step/zed_box_mini.step, which assembly_page.py writes in
this frame with its base sheet at z = 0, from Stereolabs' 85 MB STEP (kept out of git). The adapter's own screws and
inserts are checked on that page and left out here.

Ball head poses come from solving pan and spin numerically so the camera points where the
table says; `pan` also keeps the wing knob off the adapter and ZED Box Mini.
```

## Camera bracket assembly: modelling conventions

### `zed_box_mini_mount/zed_x_one_s_assembly.py`

Line 202, `above ("quarter_insert", "ball_head"),`:

```text
The stud is 1/4-20 major diameter; the insert is modeled at its minor bore.
```

## Camera bracket

### `zed_box_mini_mount/zed_x_one_s_bracket.py`

Docstring at line 1:

```text
ZED X One S bracket: holds the camera by its Stereolabs front mount on a 1/4-20 ball head.

Function: the camera hangs from its front mount (ACC-151300, zed_x_one_s_front_mount.py),
and the front mount bolts to this bracket's two cheeks through its 2 x Ø5.4 slots. The
bracket's bottom face seats on a SmallRig 2948 ball head's platform
(smallrig_2948_ball_head.py), which threads into any 1/4-20 hole on the INNOREL CP10 cheese
plate.

- Process: FDM, PLA, printed bottom face down. No supports: the cheeks rise straight off the
  floor, and the only horizontal holes are the Ø5.2 insert holes.
- Front mount: its back face (camera side) bears on the cheek fronts at y = plate_y, and its
  bottom edge rests on the floor's front ledge, so the camera height repeats. The slots still
  allow ±5 mm for the screws, which only clamp.
- Front mount screws: 4 x McMaster 91292A117 (M4 x 0.7 x 12 mm 18-8 SHCS) from the front,
  2 per slot, each on a McMaster 93475A230 washer (M4, 9 mm OD, 0.8 mm), into 4 x McMaster
  94180A353 (M4 x 0.7 brass tapered heat-set, 7.9 mm long, for a 0.208" = 5.28 mm max hole).
  The washer matters: a bare Ø7 head bears on only 0.8 mm each side of the Ø5.4 slot. The
  4 mm plate and washer leave 7.2 mm of thread, 91% of the insert, with the tip 1.8 mm off
  the hole floor.
- Ball head: 1 x McMaster 93365A160 (1/4-20 brass tapered heat-set, 0.300" = 7.62 mm long,
  for a 0.349" = 8.86 mm max hole), pressed from the top so the platform's clamp load
  wedges it into its taper. The 5.5 mm stud then engages 5.1 mm of it.
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same 11 features.
- Camera field of view (110° x 80°, wide lens) is clear: every part of the bracket sits
  behind the plate, and the screw heads sit behind the lens's FOV cone vertex.

Coordinates: origin on the 1/4-20 axis in the bottom face, +Z up (print direction), +Y the
camera's viewing direction, X across. The optical axis runs along y at (0, axis_z).

Open questions
- The ball head's dimensions are photo-scaled (smallrig_2948_ball_head.py). Its platform is
  about Ø26, which the 50 x 32 bottom face covers with margin either way.
- Insert holes are McMaster's maximum hole sizes, rounded down; test-fit an insert before
  printing the bracket, since printed holes run 0.1 to 0.2 mm under.
```

## Camera bracket dimensions); plate_t, plate_h, slot_x from front_mount.Params(

### `zed_box_mini_mount/zed_x_one_s_bracket.py`

Line 72, `width: float = 50.0`:

```text
matches the front mount's 50 mm length
1/4-20 insert 7.62 + 0.38 skin under its tip (through hole)
rear edge; 32 mm floor deep covers the Ø26 platform
front mount back face; puts the camera's mass over the stud
front mount, datasheet p.11
front mount, datasheet p.11
camera half width 12.5 + 0.7 clearance
cheek depth 18 mm behind the plate
covers the top insert (axis_z + 5 + 4.2 boss radius)
front mount slots, datasheet p.11
2 screws per slot at axis_z ± 5, inside the 20 mm slot
94180A353: 0.208" max hole
94180A353
insert + 1 mm, fasteners.md
93365A160: 0.349" max hole, drill S = 0.348"
93365A160: 0.300"
knurl OD, McMaster drawing range
floor's vertical corners
cheek back edges
elephant's foot, dfm/fdm.md
```

## Camera bracket: fillets

### `zed_box_mini_mount/zed_x_one_s_bracket.py`

Line 122, `above {"op": "fillet", "name": "Cheek backs", "size": p.cheek_r, "tangent": False,`:

```text
Cheek back edges only; the cheek fronts stay sharp where the plate seats.
```

## Front mount: store page vs drawing, notch estimate

### `zed_box_mini_mount/zed_x_one_s_front_mount.py`

Docstring at line 1:

```text
Stereolabs ZED X One S front mount (ACC-151300): 50 x 30 x 4 mm aluminum plate.

Reference model for the camera bracket assembly, not a part to make. Dimensions come from the
"ZED X One S - Front mount, Mounting Guide" page of the ZED X One S datasheet Rev1.8
(vendor/zed_x_one_s_datasheet.pdf, page 11):

- 50 x 30 x 4 mm, 4 x R4 corners
- 2 x Ø5.4 slots at x = ±18 (36 mm pitch), 20 mm center to center along the 30 mm side
- 4 x Ø2.4 thru on the camera's 21 mm square, counterbored Ø4.4 x 2 from the front
- Ø20.5 bore on the optical axis for the lens ring

The camera's front face sits on the back face; its 4 x M2 x 0.4 front holes take the screws
from the counterbored side. The store page lists the plate as 27.22 x 55 x 4 mm, which matches
neither the drawing nor the product photo, so the drawing wins.

Coordinates: origin on the optical axis in the back face (the camera side). +Y is the viewing
direction (the plate runs y = 0..4), X along the 50 mm side, +Z up.

Open questions
- The side notches on the 30 mm edges are drawn but not dimensioned; modeled 9 x 1.5 mm from
  the drawing's scale. They touch nothing in this assembly.
```

## Workarounds

- `zed_box_mini_mount/assembly_page.py:write_mesh`: each mesh is one JSON file, base64 int16
  positions in units of `MESH_MM_PER_STEP` mm (Z up) and uint32 triangle indices. Artifacts serve
  JSON but not GLB, and the viewer computes the normals.
