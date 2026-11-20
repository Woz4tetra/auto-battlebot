# synthgen cage specs

`training/synthetic/synthgen` reads the per-venue cage specs in `training/synthetic/cage/`. Only keys whose name does not say everything are listed.

## `training/synthetic/cage/meatball_basement.toml`

The meatball basement drive-test box, seen from our own ZED on a stand beside it. Units are metres in the W frame: plywood floor centre at the origin, z up, +y away from the camera, the stone-wall corner at +x +y. How each number was measured is in `playground/basement_scene/README.md`.

The geometry was measured from ZED depth of 2026-04-19T17-01-18 frame 4560: the 1.52 m plywood floor (a 5 ft square), the 2x4 rails and their height, the two stone walls and the concrete floor below. The lights and the albedo gains started as guesses and are for the grade sweeps to move. The 2026-09-19 light sweep that set `albedo_gain`, `ambient_strength` and the light height is in `docs/experiments/perception/2026-09-19_basement_light_sweep.md`.

| Key | Meaning |
| --- | --- |
| `[mat] thickness` | The rail's outer face runs from 0.088 m down to -0.035 m in the depth, not to the 3/4 in plywood's underside, because the floor sits on a frame. The mat slab stands in for that frame, and `frame_rails` hangs the rails down to its underside. |
| `[mat] albedo` | Built by `training/synthetic/build_cage_floor_texture.py --fill mirror` from the robot-patched plate of 2026-04-19T17-01-18 frame 4560: 91.5% observed, with the strip behind the near rail mirrored in. |
| `[mat] albedo_gain` | The texture is a photo of the lit plywood, so it is brighter than the plywood's reflectance. 0.9 comes from the light sweep. |
| `[cage] interior`, `wall_height` | A 5 ft floor plus a 1.5 in rail each side. No walls: the box is open above the rails. |
| `[frame] height_above_mat` | Pine 2x4s on edge, 0.0875 m above the plywood: the depth's rail-top band of 3.5 in, less the few millimetres the floor sits proud. Screwed from outside, so `bolt_pitch = 0.0` and no bolt heads show. |
| `[frame] color` | Painted over the set's colour map, because untinted Wood027 is a dark stain. The grain comes from its normal map and the colour from here. |
| `[venue] floor_drop`, `riser` | The box stands on a stand over bare concrete 0.78 m below the plywood. The space under it is open, so `riser` is false. |
| `[[walls]]` | The two stone walls of the corner, from ZED depth: the far wall 2.5 cm behind the far rail, the right one 7 cm off the right rail, both square to the box within 1 degree. Textures come from `playground/basement_scene/build_wall_textures.py`. Everything else around the box is the panorama. |
| `[[blocks]]` `name = "leg"` | Four 2x4 legs from the concrete to the box's underside, 0.1 m in from each corner. They are a stand-in, not measured: the target shows the box's own support only as dark shapes behind a bucket and a bag, so the legs are there to stop the box floating. |
| `[world] hdri` | A RICOH THETA V panorama shot from inside the basement on 2024-09-01 (`simulation/assets/panoramas/drive_test_box_1.JPG`), before the box went into the corner. It is an 8-bit JPEG, so the tubes in it clip: it is backdrop and fill, and the key light is `[lights.grid]`. |
| `[world] hdri_rotation_deg` | Turns the panorama so the shelving and boiler side lands left of the box, where the real frame shows the hoist and shelves. Its sink and stone wall end up behind the camera. |
| `[world] ambient_strength` | 0.4 from the light sweep. At 0.6 the stone reads 13 L brighter than the target, at 0.3 the floor loses its fill. |
| `[lights.grid]` | Fluorescent ceiling fixtures between the joists. `height` 1.6 m above the plywood (2.4 m above the concrete) comes from the light sweep. 1.3 m left a hot spot on the floor the target does not have. 1.9 m fits the floor slightly better but is above this basement's ceiling. |
| `[exposure]` | Solved by `render_cage_view.py --auto-exposure` against the target (floor-region means). It is the whole exposure, not a base. |
