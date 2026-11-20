# 2026-11-19 synthgen cage mount ranges and render settings

Provenance for the values in `training/synthetic/config.toml` under `[[cages]]`, `[cages.mount]` and `[damage]`. The key meanings are in `docs/reference/tools/synthgen_config.md`. Lens and view numbers also appear in `docs/experiments/perception/synthetic_domain_mix_plan_2026-09-12.md`.

## Camera and lens

- The cage camera is the synthetic cage lens (118 degrees rectified), rectified at alpha 1.0 the way the C++ `Rectifier` does it, and scaled to `[output].image_width`/`image_height`. The mount ranges were judged through this lens in `pose_camera_server.py`, so the domain frames render through it.
- NHRL's own broadcast phone (`brettzone_cage_high.toml`, 95.2 deg horizontal) belongs only on footage actually shot from the broadcast mount.
- The deployed ZED is in `config/cameras/zed2i_720p.toml`, measured at 101.0 deg horizontal. Swap it in for a ZED-matched set.
- Alpha 1.0 keeps every sensor pixel and leaves 36 percent of the frame black for this lens.
- Warped views (`distorted`, `rectified`) render a frame wide enough to hold every sensor ray: 2560x1442 for a 1280x720 frame with this lens, four times the pixels. Before 2026-09-13 every render was `pinhole`.
- The MassD and meatball halves use the same synthetic cage lens as the NHRL half. The MassD scene was fitted from a 62 degree broadcast camera, which lives in `training/data/environments/massd_arena/camera/` and is what `render_cage_view.py` grades. The meatball scene was fitted through the ZED.

## Render samples

- `render_samples = 64` with the spec's OptiX denoiser scored 37.6 dB PSNR / 0.960 SSIM against a 512-sample reference on 2026-09-12.
- 128 samples with the compositor denoiser scored 39.9 dB / 0.972.
- Robot crops at 3x were indistinguishable between the two.
- 32 samples lost faint texture on dark surfaces.

## House bot pose

- Before 2026-09-19 the house bot stood at the spec's fitted corner pose, (0.92, 1.02) facing the camera, in every frame. `randomize_house_bot` now draws a new position and heading per scene (`synthgen/house_bot_pose.py`).
- `house_bot_clearance_m` is measured between footprint circles, with each robot's radius taken as 0.20 m.

## NHRL cage mount

Source: 12 poses flown by hand in `pose_camera_server.py` on 2026-09-12 through the synthetic cage lens.

- Eight poses sit within 0.17 m of the glass. Four sit 0.35 to 0.70 m back. The ranges cover the near eight only.
- The far four frame worst of all twelve: 12 to 33 percent of the picture is mat, against 24 to 44 for the near ones.
- Including them cost more than their own share. Sampling each axis independently over the full envelope drew past -0.17 m in 78 percent of frames where the marks are there in 33, and paired heights and setbacks nothing was flown at into a derived tilt of up to 67 degrees. That rendered a camera too far back and aimed too high: 24.9 percent mat fill against the marks' 31.8.
- Over the ranges in the file: 35.1 percent fill, median tilt 40.4 degrees, 99.3 percent of the mat in frame.
- The fill number projects the mat and takes a convex hull with no occlusion term. It cannot see a pane covering the view and keeps rising as the camera closes on the glass. The near end of `inset_m` is what stops it.
- The quantities a wall swap mirrors (`along_m`, `yaw_deg`, `roll_deg`) are symmetrized, since their asymmetry says more about twelve hand-flown samples than about the mount.
- The earlier block bracketed the eleven poses `fit_cage_camera.py` solved for NHRL's own phone, a different camera on a different side of the glass. `playground/cage_scene/fit_cage_camera.py` still has those.
- Every marked pose sits outside the polycarbonate, which is why `inset_m` is negative where the phone-fitted block was positive. A pane the camera looks through from outside is hidden for that frame and costs no labels. A mount inside the cage shooting through a far pane loses them, because the segmentation pass stops at the glass.
- Each mark sits 5.1 to 18.8 degrees steeper than a straight aim at field centre (mean 10.9, sd 3.9), so every one is aimed short of the centre. That puts the far edge of the mat high in the frame and leaves foreground below it. This is why `aim = "centre"` with a negative `tilt_offset_deg` is used.
- Height is not capped by the 1.22 m polycarbonate top, since the mount is outside it.

## Near end of inset_m (render-cost cliff)

- The pane the camera looks through is hidden, but the panes either side are not. At this lens they fill much of a 118 degree frame edge-on when the camera is close. Rough refraction through a grazing slab never converges, so the adaptive sampler runs to the sample cap on most of the picture.
- Measured 2026-09-12: a scene at -0.03 took 402 seconds per render pass at 32 samples, where a healthy scene takes 4 to 23 at 128. The frames show an adjacent pane covering half the picture.
- A run that sampled -0.072 to -0.172 across 11 scenes had no slow pass. One that sampled -0.028 and -0.055 was slow on every pass. The cliff sits between 0.055 and 0.072, and -0.08 is the NHRL near end with margin.

## MassD mount

- The 2026-09-12 coverage sweeps put the mount 0.32 to 0.76 m outside the wall, because a camera on the MassD wall could not contain the 2.26 m floor at any height. That held for the lens the sweeps ran, NHRL's 95.2 degree phone.
- It does not hold for the synthetic cage lens, 118 degrees rectified. Sampled over the block in the file, the phone holds a median 91.2 percent of the floor and never the whole of it. The synthetic cage lens holds 99.0 percent and fits the whole floor in 27 percent of frames. So the camera comes in.
- The old setback made the arena a third of the picture.
- The wall plane is at 1.168 m. The glass was raised from 0.66 m to 1.41 m on 2026-09-13, above the whole height range, so every mount looks through a pane rather than over it. The pane on the camera's side is hidden whenever the camera stands outside it, so the labels behind it survive.
- `aim = "centre"` derives tilt from where the mount landed instead of drawing it independently: the sweep varied tilt with distance and height, and the frames judged good all sit on that curve. The offset is negative for the same reason as on the NHRL half.
- `inset_m` is held off the glass for the same reason as the NHRL half, where a camera within 0.055 m of the wall cost 402 seconds per render pass instead of 20. This arena's panes are 9 mm thick, and at 1.41 m tall the glass spans every mount height in the range, so the inset is what keeps every mount clear of it.
- Over this block (measured with the old 0.66 m wall) the floor fills 36.1 percent of the frame at a median 47.4 degrees off straight down, against 17.9 percent at 52.8 for the sweep block it replaces.

## Meatball basement mount

- Fitted from our own ZED on 2026-09-19 (`playground/basement_scene/README.md`).
- The box is a 1.52 m plywood floor in 2x4 rails, in a corner of stone walls.
- `mat_margin_m = 0.15` leaves a 1.22 m square of the 1.52 m floor to place robots on.
- The mount brackets the stand the ZED stood on in the fitted frame: 0.56 m outside the near rail, 0.56 m above the plywood, 64 degrees off straight down, square to the box, 3.6 degrees steeper than a straight aim at the floor centre. `inset_m` is outside the rail, and the fitted stand is at -0.56. The wall plane is the rail's outer face, 0.80 m from the floor centre.
- Only the near and left walls are used. The far and right rails have stone 2.5 and 7 cm behind them, so a mount there would sit inside the wall.
- The cage is off in the default mix (`enabled = false`, `probability = 0.0`) so existing runs keep their ratios. `config_cage_meatball.toml` pins a run to it with `only_cage`.

## Damage scene probability

- A 100-frame probe on 2026-09-12 rolled 4.33 robot-like instances per frame. Rolling only per instance at 0.35 left 15 percent of frames fully clean, not the half the damage-off arm needs. `scene_probability = 0.5` makes the split exact.
