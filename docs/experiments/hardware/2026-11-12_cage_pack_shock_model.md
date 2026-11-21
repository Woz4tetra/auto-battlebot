# Cage pack shock model, 2026-11-12

Assumptions behind `hardware/cage_pack/dynamics.py` and `hardware/cage_pack/two_sheet.py`.

## Shock and vibration model

```text
Mass budget and shock / vibration check for the cage pack on two SmallRig 4122C cups.

    venv/bin/python hardware/cage_pack/dynamics.py            # writes out/dynamics/

Model
- Outer polycarbonate sheet of the NHRL 3 lb cage wall: one 4' x 8' sheet, 1/4" thick
  (NHRL wiki, Cage Design). Linear Kirchhoff plate, simply supported on all four edges, modal
  superposition over m = 1..20, n = 1..10 (to ~600 Hz), 3 % modal damping. The cage holds
  every sheet in channels on all four sides, so the edges slide in plane and the sheet bends
  without membrane stiffening. That is the case that pries the cups hardest.
- The pack: a rigid body with three DOF at the midpoint between the cups (normal
  translation w_p, rotation theta_x about X and theta_y about Y), CG out from the glass by
  h. It couples to the plate only through the two cups (cage_layout.CUP_X, CUP_Y: one off
  each side wall, level). Each cup is a normal spring k_cup and, as a disk on an elastic
  foundation of radius R_CUP, a rocking spring k_cup R^2 / 4 about each in-plane axis
  against the sheet's slope under it, all with 10 % damping. Shear stiffness is taken as
  rigid, so in-plane loads come from the pack's rotation (a_y = -h theta_x'').
- Each cup's pull is its normal force plus its rocking moment as edge peel on the same
  disk: P = T + 4 M / R, the force that would load the seal's rim as hard if spread
  evenly. Two level cups carry the gravity moment and any rocking that way, since they
  have no vertical spacing to make a couple from.
- Excitation: a hit on the inner sheet reaches the outer sheet when the inner sheet slaps
  it across the 1" air gap. two_sheet.py models that contact for a robot hit and gives the
  momentum that arrives; here it lands on the outer sheet as a half-sine force pulse. The
  spinner bite keeps a 50 % guess: a point mass is no model of a blade biting.
- Channel engagement: two_sheet.py also gives how far each sheet's edge slides out of its
  channel, against ENGAGE (10 mm, assumed).
- Cup stiffness is unknown until the cups arrive, so every case runs over K_SWEEP and the
  page's slider picks one.
- Cup capacity: SmallRig rates 50 kg "horizontal mounting" (pull off the surface) and 35 kg
  "vertical mounting" (shear), B&H spec table. Checked as a linear interaction
  T/T_rated + V/V_rated <= 1 per cup, with the static gravity loads included.

Not modelled, and why
- The pack's effect on the sheet-to-sheet contact: two_sheet.py runs without it.
- Frame base motion: a 1.36 kg robot at 10 m/s into a cage of 150+ kg of steel moves the
  frame by under 0.1 m/s; the sheet's own flexing dominates near the pack.
- Glass-to-cup slip and vacuum leak-down over time.
```

## Panel material and stiffening

```text
Polycarbonate, Covestro Makrolon typical (E 2.38e9 Pa, nu 0.37, rho 1200 kg/m^3), 1/4" thick,
8' along the wall, 4' tall.
Berger membrane stiffening: a uniform in-plane tension N from the sheet's stretch,
N = E h / (2 (1 - nu^2) A B) * integral |grad w|^2, edges held in plane. Each mode then
gains stiffness N k_i^2 (A B / 4). Off when berger is False (linear upper bound).
```

## Static cup loads

```text
Gravity on the pack, per cup: normal pull T, rocking moments, shear, and the pull P
with edge peel. The cups share the gravity moment M g h by their stiffness (k cancels).
```

## Pack model

```text
Pack inertia about its own CG, box ~ 160 x 100 x 100 mm.
DOFs: [q_1..q_nm, w_p, theta_x, theta_y]; pack normal motion at (dx, dy) from the
reference is w_p + theta_x dy + theta_y dx.
Each cup: an axial row (stretch w_p + theta_x dy + theta_y dx - phi . q) and two
rocking rows (pack rotation less the sheet's slope under the cup).
Rocking about X: the cups' vertical spacing as a couple plus each one's own disk.
```

## Two-sheet model

```text
Two-sheet cage wall impact: robot -> inner sheet -> 1" air gap -> outer sheet.

    venv/bin/python hardware/cage_pack/two_sheet.py --v 10 [--held] [--hit x,y]

Gives the momentum that reaches the outer sheet, which dynamics.py takes as its input,
and how far each sheet pulls out of its edge channels.

- Both sheets: 1/4" PC, 2.44 x 1.22 m, simply supported, modal (sin x sin) to FMAX = 600 Hz,
  3 % modal damping. The cage holds them in channels on all four sides, so the edges slide
  in plane: plain linear bending, no membrane stiffening. --held adds Berger stiffening
  (edges pinned in plane) for comparison.
- Robot: 1.36 kg point mass with a compression-only contact spring on the inner sheet.
- Sheet to sheet: compression-only penalty springs on a 61 x 31 grid, 25.4 mm gap.
- Channel pull-in: an edge slides in by half the arc-length excess of the bent sheet across
  that span, 1/2 * integral of slope^2, split between the two edges; the worst line across
  the sheet is reported.
- Semi-implicit Euler on modal coordinates, dt 10 us; halving dt changes results under 1 %.

Berger: N = (3 D / (2 h^2)) sum q^2 k^2 (tension per unit length, both directions)

Channel pull-in. Along Y at column x: 1/2 int w_y^2 dy = 1/2 sum_n (n pi/B)^2 (B/2) a_n(x)^2,
a_n(x) = sum_m q_mn sin(m pi x / A); the same along X. Sampled on 41 lines each way.

Outer bending stress at the surface: sigma = 6 M / h^2, plus membrane N / h.
```
