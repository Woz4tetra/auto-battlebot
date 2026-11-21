# synthgen cage specs

`training/synthetic/synthgen` reads the per-venue cage specs in `training/synthetic/cage/`. Only keys whose name does not say everything are listed.

## `training/synthetic/cage/meatball_basement.toml`

The meatball basement drive-test box, seen from our own ZED on a stand beside it. Units are metres in the W frame: plywood floor centre at the origin, z up, +y away from the camera, the stone-wall corner at +x +y. How each number was measured is in `training/synthetic/scenes/basement_scene/README.md`.

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
| `[[walls]]` | The two stone walls of the corner, from ZED depth: the far wall 2.5 cm behind the far rail, the right one 7 cm off the right rail, both square to the box within 1 degree. Textures come from `training/synthetic/scenes/basement_scene/build_wall_textures.py`. Everything else around the box is the panorama. |
| `[[blocks]]` `name = "leg"` | Four 2x4 legs from the concrete to the box's underside, 0.1 m in from each corner. They are a stand-in, not measured: the target shows the box's own support only as dark shapes behind a bucket and a bag, so the legs are there to stop the box floating. |
| `[world] hdri` | A RICOH THETA V panorama shot from inside the basement on 2024-09-01 (`simulation/assets/panoramas/drive_test_box_1.JPG`), before the box went into the corner. It is an 8-bit JPEG, so the tubes in it clip: it is backdrop and fill, and the key light is `[lights.grid]`. |
| `[world] hdri_rotation_deg` | Turns the panorama so the shelving and boiler side lands left of the box, where the real frame shows the hoist and shelves. Its sink and stone wall end up behind the camera. |
| `[world] ambient_strength` | 0.4 from the light sweep. At 0.6 the stone reads 13 L brighter than the target, at 0.3 the floor loses its fill. |
| `[lights.grid]` | Fluorescent ceiling fixtures between the joists. `height` 1.6 m above the plywood (2.4 m above the concrete) comes from the light sweep. 1.3 m left a hot spot on the floor the target does not have. 1.9 m fits the floor slightly better but is above this basement's ceiling. |
| `[exposure]` | Solved by `render_cage_view.py --auto-exposure` against the target (floor-region means). It is the whole exposure, not a base. |

## `MatSpec`

`albedo_gain` and `WallSpec.albedo_gain` share the reasoning below.

| Field | Meaning |
| --- | --- |
| `albedo_gain` | Linear multiplier on the mat albedo image. The image is a photo of the lit mat, brighter than the paint's real reflectance. Darkening it lets the lights, fitted by auto exposure to the mat, carry the brightness, so every true-albedo object (the robots) is lit to match. |

## `CageSpec`

| Field | Meaning |
| --- | --- |
| `side_posts` | Extra vertical posts along each side, as fractions of the side length from its centre. 0.0 is a mid-side post. Corner posts are always present. |
| `corner_posts` | The four corner posts. Off for an open box with no walls, where nothing stands at the corners above the rail. |

## `FrameSpec`

| Field | Meaning |
| --- | --- |
| `bolt_face` | `"top"` stands the bolt heads on the rail's upper face, which is what a low steel rail shows. `"inner"` drives them horizontally through the face looking at the mat, which is what a tall wooden kick rail shows. `bolt_height_above_mat` then says how far up. |

## `PanelSpec`

| Field | Meaning |
| --- | --- |
| `walls` | Which walls to glaze. BlenderProc's segmentation stops at glass, so a wall between the camera and the mat costs every label behind it. Leave it out when the camera is outside the cage looking in. `panels_outside_camera` does this per frame for mounts outside the wall plane. |

## `PitSpec`

A rectangular hole through the mat. The mat is cut around it, with walls and a floor below.

| Field | Meaning |
| --- | --- |
| `center`, `size` | The rim in the W frame, the opening measured at mat level. |
| `wall_thickness` | The four walls sit just outside the rim, so their inner faces land on the cut edge. |
| `depth` | The floor closes the box off at this depth so nothing under the arena shows through. |

## `WallSpec`

A photo-textured vertical wall of the venue, standing on the venue floor. It runs from `start` to `end` in W at every height from `z_bottom` to `z_top`. It exists for the basement, whose stone walls stand a few centimetres behind the box, too close for a panorama at infinity to stand in.

| Field | Meaning |
| --- | --- |
| `albedo` | Image laid out the way `wall_quads` maps it: columns from `start` to `end`, rows from `z_top` down to `z_bottom`. |
| `color` | Used when `albedo` is empty. |

## `VenueSpec`

| Field | Meaning |
| --- | --- |
| `riser` | The solid block from the mat down to the venue floor. Off for a box standing on sawhorses, where the space under it is open. |

## `HouseBotBoxSpec`

| Field | Meaning |
| --- | --- |
| `size` | The GLB is scaled so its footprint's longer side equals `size[0]`. 0 keeps the model's own size. `size[2]` is unused when scaling. |
| `roll_deg` | Rotation about the model's own x axis (the LED-face normal), applied before yaw. The GLB lies on its side, so -90 stands it up with the eyes side by side and the mouth below. |
| `front_fill` | Strength of a narrow spot on the camera side aimed at the box, so its vertical LED face is lit the way the venue lights light the real one. 0 disables it. |
| `model` | Textured model, repo-relative. An empty string falls back to a flat-coloured box. |
| `albedo_gain` | Linear scale on the texture's grey body. 1.0 matches the mat brightness, as the real box does. |
| `led_emission` | The LED eyes and mouth in the texture glow: saturated texels emit at this strength. |

## `WorldSpec`

| Field | Meaning |
| --- | --- |
| `hdri` | Repo-relative equirectangular panorama. When set, it replaces `background_color` as the world, at `ambient_strength`, turned `hdri_rotation_deg` about z. |

## `TubeGrid`

A rows x cols array of identical LED tubes, the ceiling rig as one knob. Shadow softness follows the rig's extent and tube size: more, longer, thicker tubes spread further apart give softer shadow edges. Rows run along y, columns along x.

## `LightsSpec`

| Field | Meaning |
| --- | --- |
| `tubes` | Extra tubes beside the grid. |

## `BackdropSpec`

| Field | Meaning |
| --- | --- |
| `enabled` | The cylinder and its light strip. Off when the world panorama is the backdrop. |

## `training/synthetic/cage/cage2_overhead_high.toml`

The NHRL 3 lb cage (Cage 2) as seen by the fixed Cage-2-Overhead-High iPhone. Units are metres in the W frame: mat centre at the origin, z up, +y away from the camera, +x to the camera's right. Unknown dimensions started as guesses and are tuned by `training/synthetic/scenes/cage_scene/grade_render.py` sweeps. The values are the current best, not measurements.

| Key | Meaning |
| --- | --- |
| `[mat] albedo` | Built by `training/synthetic/build_cage_floor_texture.py`, in the hfield frame. The `.json` beside the PNG holds the frame. |
| `[mat] albedo_gain` | Linear multiplier on the albedo photo. Below 1 the lights carry more of the mat brightness and the robots come out brighter for the same mat match (see the robot-frame sweep). |
| `[panel] tint`, `roughness` | Below clear glass: the real panels are scratched and dusty, and what the camera sees through them is darker than the venue lighting alone would make it. |
| `[venue] floor_cc_texture` | `"none"` keeps the flat `floor_color`. A cc_textures set name swaps in its PBR maps. |
| `[house_bot_box]` position | The May 2026 framing shows the house bot parked on the far-right rail. The June framing has it near-left. |
| `[house_bot_box] size` | The textured GLB is scaled so its footprint's longer side is `size[0]`. `size[0] = 0` keeps the model's own 0.25 x 0.23 x 0.33 m. `color` and `roughness` are the fallback flat box when the GLB is missing. |
| `[house_bot_box] roll_deg`, `yaw_deg` | The model ships on its side. Roll -90 about its x axis (the LED-face normal) stands it up. The face is the +x side. Yaw -90 turns it toward the camera and -80 matches the real angle. |
| `[house_bot_box] front_fill` | Camera-side area light on the box so its vertical face is lit like the real one. |
| `[lights.grid]` | Overhead LED tubes, the key light: a rows x cols rig of identical tubes running along x. The rig footprint (`spacing_x`, `spacing_y`, `height`) sets where the cage casts its shadows. The tube size (`length`, `radius`) sets how crisp the robot shadows are: shorter, thinner tubes give sharper edges. Extra explicit tubes can be added as `[[lights.tubes]]` entries. |
| `[[lights.wash]]` | Coloured wash behind the panels: magenta on the camera's left, cyan on its right. |
| `[exposure]` | Solved by `render_cage_view.py --auto-exposure` against the May 2026 targets (mat-region means). It is the whole exposure, not a base: `--auto-exposure` multiplies it by what it solves, so a run without it renders the graded look. Anything that applies the spec as-is (`render_cage_samples.py`, the `[cage]` half of `render_scenes.py`) depends on that. The LED emission is divided by this gain so the house bot lands at the same output brightness. |

## `training/synthetic/cage/massd_resurgence6.toml`

The MassDestruction arena (Charles River Museum, Resurgence Six) as seen by the Omega broadcast camera. Units are metres in the W frame: floor centre at the origin, z up, +y away from the camera.

Two dimensions are measured: the 2.26 m plywood floor (an 8 ft wall interior less a 2x4 rail each side, confirmed against our own metric ZED recording of the same arena), and the two 0.43 m hazard squares at (0, +-0.88), read straight off the rectified floor texture. Everything else started as a guess and was moved by the grade sweeps.

| Key | Meaning |
| --- | --- |
| `[mat] albedo` | Built by `training/synthetic/build_cage_floor_texture.py` from the empty frame 8140 of `r1_beeroll_vs_mrsbuff`, in the hfield frame (see the `.json` beside it). 89% of the floor is observed and the near corner is filled with the floor mean. |
| `[mat] albedo_gain` | The albedo is a photo of the lit plywood, so as a reflectance it is far too bright. Below 1 the lights carry the floor brightness and the robots come out lit to match. Fitted against the real robot's brightness; see [the Resurgence 6 cage fit](../../experiments/perception/2026-09-13_massd_resurgence6_cage_fit.md). |
| `[cage] wall_height` | Raised 0.75 m on 2026-09-13, from the 0.66 m fitted against the r1 footage, for taller glass walls than Resurgence 6 had. Sets the corner posts and the top edge of every pane. |
| `[cage] post_size` | Small: the riser under the arena is drawn `post_size` wider than the wall, and anything wider than the kick rail shows up as a lit ledge the real arena does not have. |
| `[frame] height_above_mat` | The rail is a wooden kick board standing on edge at the floor line, not a ledge: the far side of the picture shows its inner face and the near side only its top edge. Its height is solved from the footage; see [the Resurgence 6 cage fit](../../experiments/perception/2026-09-13_massd_resurgence6_cage_fit.md). Carriage bolts sit every 16 in through the face. |
| `[[pits]]` | The near pit is the one the camera looks into. The far square reads flat in this footage (dark grey with a red chevron painted on it), so it stays part of the floor texture. Its depth is solved from the footage; see [the Resurgence 6 cage fit](../../experiments/perception/2026-09-13_massd_resurgence6_cage_fit.md). |
| `[panel]` | Scratched polycarbonate: the crowd behind it reads darker and greyer than the room. All four walls are glazed. A pane the camera looks through from outside the cage is hidden for that frame (see `panels_outside_camera`), because BlenderProc's segmentation stops at glass and a pane in front of the mat would cost every label behind it. Panes seen from inside, across the arena, render normally. |
| `[world]` | Almost no flat ambient. The grade wants the light to come from above and nowhere else: raising the ambient lights the outward-facing faces of the near rail, which the real picture shows almost black, and costs 10 points of outside L1 for nothing inside. |
| `[lights.grid]` | Overhead hall fixtures. The rig was picked by a sweep against the target; see [the Resurgence 6 cage fit](../../experiments/perception/2026-09-13_massd_resurgence6_cage_fit.md). |
| `[backdrop]` | The standing crowd and the rest of the hall, close in behind the glass on every side. |
| `[exposure]` | Solved by `render_cage_view.py --auto-exposure` against the r1 target (floor-region means). It is the whole exposure, not a base. |
