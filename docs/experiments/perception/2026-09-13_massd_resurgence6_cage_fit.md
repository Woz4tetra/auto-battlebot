# Resurgence 6 cage fit

Values in `training/synthetic/cage/massd_resurgence6.toml` that were fitted or solved against the
`r1_beeroll_vs_mrsbuff` footage. Key meanings are in
[`synthgen_cage_specs.md`](../../reference/tools/synthgen_cage_specs.md).

## Floor albedo gain

`[mat] albedo_gain = 0.50` puts the rendered MRS BUFF's brightness on the real one over 17
hand-labelled frames of the clip.

| `albedo_gain` | Lab dL (render minus real) |
| --- | --- |
| 0.35 | +9.4 |
| 0.50 | -0.3 |
| 0.90 | -6.7 |

## Kick rail height

`[frame] height_above_mat = 0.095` m is solved from how far the rail's bright band reaches above
the floor edge. It reads the same within 5 px at four points along the far edge.

## Pit depth

The near pit's depth, 0.13 m, is solved from how much of the pit floor the near wall hides.

## Light rig

A 2x2 grid at 1.4 m beat every rig from 1.2 to 3.0 m and every count from 2x2 to 3x3: full-frame
L1 13.3, against 13.4 to 19.8 for the others.
