# Synthetic domain mix: how much cage data, and does randomized still earn its place

Plan for generating 20,000 NHRL-cage and 20,000 MassD-arena synthetic frames, then training
`yolo26x-pose` on ratios of those against the existing corpus. Four questions, one render
budget, one eval set.

Writeup lands in `docs/experiments/perception_performance/synthetic_domain_mix_<date>.md`.

## Status, 2026-09-12

Steps 0, 1 and 2 are done. Step 3 has not started. The 100-frame inspection renders for both
venues are queue jobs 42 and 43 on megamind, writing `training/data/_probe_nhrl_2026-09-12`
and `training/data/_probe_massd_2026-09-12`.

| Step | State |
| --- | --- |
| 0a lens | Done, and it changed the answer |
| 0b timing | Local only: 9.3 s per written frame at 128 samples on an RTX 4080 Laptop. No A6000 number yet |
| 0c schema | Done. `nc: 4` lowercase, straight out of the renderer |
| 1 megamind | Assets, image and code staged. 61 GB free on `/` |
| 2 damage | Done, with three changes to the design |
| 3 render | Not started. Needs `render_shards.sh` and the `CUDA_VISIBLE_DEVICES` passthrough |

### What 0a settled

The deployed ZED reads 1280x720, fx = fy = 527.528, cx 644.906, cy 369.885, zero distortion,
across 17 saved MCAPs from MassD_2026-08-29 and NHRL_2026-05-02. That is in
`config/cameras/zed2i_720p.toml`, and it is 101.0 degrees horizontal against the phone's 95.2:
a 10 percent difference in normalized focal length, so the two lenses are not interchangeable
and the question was worth asking.

Both venues render through `config/cameras/ecam25_h01r1_estimated.toml` anyway, because the
e-CAM25 is the camera the mounts are for and the lens they were judged through.
`zed2i_720p.toml` stands as the measured record for a ZED-matched set later.

HD720 is not a scaled HD1080. The one 1080p recording in the corpus reports fx/width 0.554
against 0.412 here, so the sensor crops where HD720 bins. Do not rescale that file.

### Mount ranges

`[cages.mount]` for `nhrl_cage` now comes from 12 poses flown in `pose_camera_server.py` and
covers the near-glass eight of them:

```toml
walls = ["near", "far", "left", "right"]
along_m = [-0.55, 0.55]
height_m = [0.60, 1.25]
inset_m = [-0.20, -0.01]
aim = "centre"
tilt_offset_deg = [-22.0, -10.0]
yaw_deg = [-23.0, 23.0]
roll_deg = [-13.0, 13.0]
```

Every marked pose sits outside the polycarbonate, which is why inset is negative where the
phone-fitted block was positive. That is the cheaper side to shoot from: a pane the camera
looks through from outside is hidden for that frame and costs no labels.

Two things went wrong on the way here, both worth not repeating.

- Bug: the marks only existed in the running server's memory. The render directory under
  `--out` is not evidence they were saved.
- Fix: `curl -s -X POST -d '{"name":"..."}' http://127.0.0.1:8770/save` writes
  `mount_ranges.toml`, and `GET /state` reads the live marks out of a server still up.

- Bug: the first ranges took the envelope of all 12 marks and sampled each axis independently.
  That rendered a camera too far back and aimed too high, filling 24.9 percent of the frame
  with mat against the marks' 31.8. Uniform draws over `inset [-0.75, -0.01]` land past
  -0.17 m in 78 percent of frames where the marks are there in 33, and independent axes paired
  heights and setbacks nothing was flown at into a derived tilt of up to 67 degrees.
- Fix: the ranges above, which cover the near-glass eight. Over 2000 samples they fill 38.9
  percent of the frame with mat, median tilt 39.0 degrees, 98.9 percent of the mat in frame.
  The far-back four framed worst of all twelve at 12 to 33 percent.

A negative `tilt_offset_deg` is steeper, aimed short of the field centre, because tilt is
measured off straight down. Every mark is aimed short of the centre by 5.1 to 18.8 degrees.

### Damage, as built

`synthgen/damage.py` holds the draws and stays out of Blender, so the purity guard covers it
and 33 unit tests do. `synthgen/damage_scene.py` applies a draw and reverts it. Three changes
from the design above:

- **Nothing is deleted.** `load_robots` and the distractor pool load each model once and reuse
  it for every later scene, so a deleted part would stay deleted for the rest of the run. Part
  removal hides parts from the render, which hands both passes the same silhouette; chunk
  removal attaches a boolean difference modifier and takes a cutter from a pool built at
  startup, before `_enable_segmentation` arms the meshes that exist. A `DamageSession` reverts
  both in a `finally` after the scene's frames are written.
- **`separate loose parts` is not used.** Mesh count picks the mechanism: more than one mesh
  goes to part removal, one fused mesh goes to the cutter. Splitting a Meshy mesh is
  destructive, which the reuse rule forbids.
- **The scene split is tracked, not flipped.** Damage has to be drawn per scene, since one
  render call covers every camera pose in the scene. Rolling only per instance at 0.35 left 0
  of 30 frames fully clean on the first probe, not the 40 to 50 percent this plan assumed: a
  frame carries 4.1 robot-like instances, so `0.65^4` is 15 percent before the binomial spread
  of ten scenes. `[damage].scene_probability` now sizes the clean pool directly, and
  `DamageBudget` tracks it the way `choose_cage` tracks the scene mix, so the ratio holds to
  within one scene at any run length.

The two mechanisms measure geometry in different frames, on purpose. The cutter is an
unparented world object, so chunk removal works in world space: a distractor's parent carries
a per-scene scale, and a radius measured in the parent frame comes out wrong by that factor.
Part removal works in the parent frame, because that is where the parts and keypoints were
modelled, so protection does not depend on which way the robot faces. Measured over three
poses, one robot's protected set held at 9 and 36 parts in its own frame while world-space
boxes wobbled between 6 and 71.

At `scene_probability = 0.5` the first 100-frame pair came out 60 percent clean (NHRL) and 50
percent (MassD). NHRL runs high because a scheduled damaged scene can still roll every
instance clean. A 20k render therefore yields 10k to 12k clean frames, so a damage-off arm
cannot match a 20k damage-on arm at the same count. Drop `scene_probability` or accept the
mismatch.

Grading the mechanism: part removal on the CAD robots reads as missing armour with internals
showing. The cutter takes a plausible bite out of a Meshy shell, but the cut exposes unshaded
backfaces that render bright white, which no real robot looks like. A dark interior material
on cut faces would fix it. At 34 to 70 px it may not matter.

### Findings that affect later steps

- **The domain frames put our robots at roughly half the pixels the randomized pool does.**
  Longest bbox edge at 1280x720, median: `mr_stabs_mk2` 39 px (NHRL) and 34 px (MassD) against
  58 px randomized; `mrs_buff_mk3` 70 and 52 against 96. That is faithful to what a cage mount
  sees, but step 4 pins `imgsz 640` for every arm, and 34 to 39 px is the size range that
  already starved the cage-high detector. A domain arm can lose on resolution and read as
  losing on domain. Pre-register imgsz as a factor or record the confound.
- **Keypoint visibility did not move.** 3.9 percent flag-0 on the NHRL domain probe against
  5.2 percent in the randomized pool, so the outside-the-glass mounts and the one-way panes
  are not eating keypoints.
- **`validate_yolo_integrity.py --strict` cannot pass on a MassD render.** It reports zero
  errors and one warning, `house_bot` with zero instances, which is correct: that spec has
  `[house_bot_box] enabled = false`. The step 3 gate needs this written in as an exception.
- **Each written frame costs two render passes.** The clean distractor-free pass doubles every
  render. `[output].ignore_obstructions = true` halves the render cost and drops the occlusion
  gate with it, which is the biggest single lever on a multi-day render.
- **Output is smaller than budgeted.** 130 to 140 KB per frame, so 40,000 frames is about 5.4
  GB rather than 8.

### Two config mechanisms added

`extends` and `only_cage`, both in `synthgen/configuration.py`. A per-venue config is three
keys different from the shared one, and copying 1000 lines twice would mean editing every
`[[robots]]` change three times. `config_cage_nhrl.toml` is six lines:

```toml
extends = "config.toml"
only_cage = "nhrl_cage"

[output]
num_images = 20000
```

Tables merge key by key; arrays, including `[[robots]]`, replace wholesale. `extends` must name
a file in the same directory, because relative paths inside the inherited config resolve
against the loaded file. `only_cage` puts the named cage at probability 1.0 and disables the
rest, so no scene lands in the HDRI arena or the other venue.

### Left to do

1. The A6000 timing probe, at 128 and 64 samples. It needs a per-cage `render_samples`
   override, not `--render-samples`.
2. Decide on the white interior faces the cutter exposes.
3. Decide imgsz for step 4, given the pixel-size gap above.
4. `render_shards.sh` and the `CUDA_VISIBLE_DEVICES` passthrough, then step 3.

## Questions

1. **Amount.** How many domain-synthetic frames before the eval curve flattens? Is 20k per
   venue overkill or not enough?
2. **Domain vs randomized.** At a matched frame count, does cage-domain synthetic beat
   randomized HDRI scenes?
3. **Do I need randomized at all?** Once domain data is in, does deleting all 17,995
   randomized frames cost anything?
4. **Damage.** Does randomized part loss on our robot and on opponent meshes improve recall
   on real damaged robots?

Question 4 rides along at no extra render cost: damage is sampled per instance and recorded
per frame, so damage-on and damage-off arms are filters over the same render, not two renders.

## What already exists

| Piece | State |
| --- | --- |
| `training/data/all_robot_keypoints` | 18,447 train (17,995 randomized synthetic + 452 real), 2,049 val. `nc: 3` `[mr_stabs_mk2, mrs_buff_mk3, nhrl_robot]`, `kpt_shape [2, 3]` |
| `training/data/synthetic` | the flat 20,001-frame randomized pool the above was split from |
| `training/synthetic/render_scenes.py` | generic pipeline, `[cage]` section renders a tracked fraction of scenes inside a cage spec at 1280x720. Has `--num-images`, `--start-index`, `--seed`, `--out`, `--render-samples` |
| `training/synthetic/cage/cage2_overhead_high.toml` | NHRL 3 lb cage, graded in `cage_scene_render_match_2026-09-11.md` |
| `training/synthetic/cage/massd_resurgence6.toml` | MassD arena, graded in `massd_arena_scene_2026-09-11.md`. Generic-pipeline integration in flight by another agent |
| `synthgen/cage_mount.py` | samples wall mounts (1.00 to 1.45 m up, 26 to 42 deg tilt, 0.02 to 0.25 m inside the glass), bracketing where our camera goes |
| `training/data/nhrl_keypoints_eval_test` | 688 `pass` frames over 8 recordings: 590 NHRL May, 98 MassD Aug. `nc: 4` `[mr_stabs_mk2, mrs_buff_mk3, opponent, house_bot]` |
| `training/yolo/make_scaling_splits.py` | writes arms as image-list `.txt` files, so arms cost kilobytes and share one disk cache |
| `training/model_eval/{score.py, edit_labels.py, make_eval_dataset.py}` | scoring against TensorRT engines, the label editor, the empty-label dataset builder |

`render_cage_samples.py` is the other cage renderer. It uses the fitted broadcast-camera poses
and belongs to the scene-grading loop, not to this experiment. Everything here goes through
`render_scenes.py` so the mount varies.

## Step 0: settle three things before rendering 40,000 frames

These are cheap and each one can invalidate the render.

### 0a. Point the cage camera at our own intrinsics

`[cage].camera_calibration` is `config/cameras/brettzone_cage_high.toml`, NHRL's phone at
95.2 deg horizontal. Our deployed camera is the ZED. If the two fields of view differ, every
domain frame is rendered through the wrong lens and the whole premise of question 2 is
weakened.

```bash
# The saved MCAPs predate the Foxglove migration, so convert one first.
venv/bin/python scripts/convert_ros1_mcap.py \
  data/saved_recordings/MassD_2026-08-29/auto_battlebot_mrs_buff_mk3_massd_ns_jetson_2026-08-29_13-08-16__2026-08-29T13-20-08.mcap \
  /tmp/massd_converted.mcap
# Then read /camera/camera_info (auto_battlebot.recording.mcap_io.decode_camera_info).
```

Write the result to `config/cameras/zed2i_1080p.toml` in the same schema
(`calibration_id`, `width`, `height`, `fx`, `fy`, `cx`, `cy`, `k1..k3`, `p1`, `p2`) and set
`[cage].camera_calibration` to it. If the ZED numbers land within a few percent of the phone,
record that and move on.

**Done 2026-09-12, with two corrections.** The MassD MCAPs are already in the current format,
so `convert_ros1_mcap.py` skips them with `profile '' is not ros1`; read them straight through
`auto_battlebot.recording.mcap_io.iter_messages`, which yields `(topic, log_time, payload)`
and tags the payload so `decode_camera_info` accepts it. And the camera runs HD720, not 1080p,
so the file is `config/cameras/zed2i_720p.toml`. See the status section for what it says and
why the renders use the e-CAM25 regardless.

### 0b. Timing probe

Nothing in the repo records seconds per frame for a cage scene, and the render is the schedule
driver. Run 200 frames per spec and measure.

The probe runs on megamind, on one A6000, after step 1 has staged the assets there.

```bash
ssh megamind
cd /home/ben/auto-battlebot
CUDA_VISIBLE_DEVICES=0 bash training/synthetic/docker/run_synthetic.sh --require-gpu \
  auto-battlebot-synthetic blenderproc run render_scenes.py -- config_cage_nhrl.toml \
  --num-images 200 --out ../data/_probe_nhrl --render-samples 128 --seed 0
```

Record wall clock, peak VRAM, and the drop rate (`scenes_attempted` vs `images_written`).
Repeat at `--render-samples 64`. If 64 grades the same on a spot check, take it: the cage
config asks for 128 because the glass is noisy, and halving samples halves a multi-day render.

**Two corrections, 2026-09-12.** `--render-samples` never reaches a cage scene: each
`[[cages]]` entry sets its own `render_samples = 128`, which overrides the run value. Testing
64 means a variant config that overrides the per-cage key. And `--require-gpu` used to set only
a flag, leaving the GPU arguments empty, so the command above took the CPU path silently
without even reaching the GPU probe. It now implies `--gpu`.

From seconds per frame, compute the full render cost three ways and pick the shard count:
40,000 frames on one A6000, on two, on all three. That number decides how long the queue is
blocked, which is the real cost of rendering on the training box.

### 0c. One class schema for every arm

Three schemas are live right now and they do not agree:

- `all_robot_keypoints`: `nc: 3`, lowercase, no `house_bot`
- cage renders: `nc: 4`, uppercase `MR_STABS_MK2`, includes `house_bot`
- eval set: `nc: 4`, `opponent` where the training sets say `nhrl_robot`

Pick `nc: 4` `[mr_stabs_mk2, mrs_buff_mk3, nhrl_robot, house_bot]` for training, lowercase
throughout. `all_robot_keypoints` needs a `data.yml` bump only, since its class ids 0 to 2
already match and no frame carries a house bot. The score call is then one string for every
arm:

```
--labels "mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot"
```

The MassD arena has no house bot, so `house_bot` rows come only from the NHRL half. Expect its
AP to move with the NHRL fraction and read it separately.

## Step 1: stage the render on megamind

Rendering moves to megamind, where the training is. That buys three A6000s and drops the
dataset transfer entirely, since the render output is already on the training box. It costs
queue time: the render has to own the GPUs while it runs, so it goes through `gpu_queue.py`
like any arm.

### What megamind has and does not have

Checked 2026-09-12:

| Path | megamind | Action |
| --- | --- | --- |
| `training/data/models` | present, 82 MB | verify the two robot files match, do not re-send |
| `training/data/distractor_models/robots` | **missing**, 3.9 GB local (146 Meshy GLBs) | upload |
| `training/data/distractor_models/distractor_gpu_audit.csv` | **missing**, 224 KB | upload |
| `training/data/environments` | **missing**, 25 MB | upload |
| `training/data/cc_textures` | **missing**, 7.8 GB local across 524 sets | upload **13 sets only**, 151 MB |
| `training/data/hdris` | **missing**, 4.5 GB local | skip |
| `training/data/distractor_models/objaverse` | **missing**, 9.9 GB local | skip |
| `training/data/distractor_models/robots_backup` | **missing**, 2.8 GB local | skip |
| `auto-battlebot-synthetic` docker image | **missing** (playback image is there) | build on megamind |
| NVIDIA container runtime | working, `--gpus all` sees all three A6000s | nothing |
| `/media/storage/auto-battlebots-archive` | present, 1.3 TB free | render output goes here when done |

Three of those skips need justifying, because each one is a multi-GB transfer avoided:

- **HDRIs.** `[cage].probability = 1.0` means no scene ever takes the HDRI arena path.
  `load_environment_assets` globs the HDRI dir only `if hdri_dir.exists()` and returns an
  empty list otherwise, so an absent dir degrades instead of raising. Confirm on the 200-frame
  probe that the log says `0 HDRIs available` and every frame still renders.
- **objaverse.** Its `[[distractors.sources]]` block is commented out in `config.toml`. The
  only live source is `../data/distractor_models/robots`.
- **Most of cc_textures.** Robot and cage materials load through
  `bproc.loader.load_ccmaterials(dir, used_assets=[...])`, so only named sets are read. The
  named sets across `config.toml`, both cage specs, and the `cage_spec.py` dataclass defaults
  are: `Concrete035`, `Foil002`, `Foil003`, `Metal012`, `Metal030`, `Paper001`, `Plastic007`,
  `Plastic007_blue`, `Plastic007_yellow`, `Rubber001`, `Wood027`, plus the two sticker dirs
  `mrs_buff_mk3_top_sticker` and `mrs_buff_mk3_bottom_sticker`. That is 151 MB, not 7.8 GB.

The one path that loads the **whole** texture dir is `load_environment_assets`, which calls
`load_ccmaterials` with no `used_assets` to build the ground-plane material list. In a
cage-only run those ground materials are never applied, so a pruned dir just yields a shorter
list. Watch the `N CC textures available for ground` line on the probe and confirm it says 13
rather than failing.

### Upload

Measured link: 100 MB in 24 s, about 4.2 MB/s. The 4.1 GB payload is roughly 16 minutes.
No `-z`, since GLBs and PNGs are already compressed.

```bash
# 1. Meshy opponent pool and the VRAM audit it is gated by.
rsync -a --info=progress2 \
  training/data/distractor_models/robots \
  training/data/distractor_models/distractor_gpu_audit.csv \
  megamind:/home/ben/auto-battlebot/training/data/distractor_models/

# 2. Cage and arena environments: mat albedos, house bot textures, camera metadata.
rsync -a --info=progress2 training/data/environments/ \
  megamind:/home/ben/auto-battlebot/training/data/environments/

# 3. Only the referenced texture sets. The `./` marks where the preserved path starts, so it
#    belongs at the repo root, not inside cc_textures: with the marker after cc_textures these
#    landed as /home/ben/auto-battlebot/Concrete035 and had to be moved by hand.
rsync -a --info=progress2 --relative \
  ./training/data/cc_textures/{Concrete035,Foil002,Foil003,Metal012,Metal030,Paper001,Plastic007,Plastic007_blue,Plastic007_yellow,Rubber001,Wood027,mrs_buff_mk3_top_sticker,mrs_buff_mk3_bottom_sticker} \
  megamind:/home/ben/auto-battlebot/

# 4. Confirm the robot models already there are the ones the config names.
ssh megamind 'cd /home/ben/auto-battlebot && ls -l "training/data/models/MR STABS MK2.gltf" "training/data/models/MRS BUFF MK3.glb"'
```

Everything else the render needs is tracked in git: `synthgen/`, the per-venue configs, the
cage specs, the Dockerfile, `run_synthetic.sh`. Push the branch and pull it on megamind.

### Build the image on megamind

```bash
ssh megamind 'cd /home/ben/auto-battlebot && \
  docker build -f training/synthetic/Dockerfile -t auto-battlebot-synthetic training/synthetic'
```

The local image is 11.3 GB and bakes Blender 4.2.1 plus both pip trees, so expect a similar
size and a 20 to 40 minute build. megamind's docker root is `/var/lib/docker` on `/`, which
has 76 GB free, so the build fits and leaves about 60 GB.

Build rather than transfer. `docker save | ssh megamind docker load` moves 11.3 GB at
4.2 MB/s, which is 45 minutes, and the Dockerfile pins its pip versions, so a rebuild is
reproducible. Fall back to save-and-load only if the build resolves different apt packages.

### Assets must live inside the repo tree

`run_synthetic.sh` mounts exactly one host path, `-v "${repo_root}:/workspace"`. A symlink
from `training/data/...` out to `/media/storage` dangles inside the container, because the
link target is not mounted. So on megamind the render assets and the render output are real
directories under `/home/ben/auto-battlebot`, not symlinks into `/media/storage`.

`/` on megamind is at 92 percent with 76 GB free. The budget: 4.1 GB of assets, about 11 GB
for the image, and about 8 GB of render output. That fits, with roughly 50 GB to spare.

When a venue's render finishes and passes its gates, move it to the archive and point the
training `data.yml` `path:` at the new location. Training reads through the venv, not through
docker, so it does not care where the dataset lives:

```bash
ssh megamind 'mv /home/ben/auto-battlebot/training/data/synth_cage_nhrl_<date> \
  /media/storage/auto-battlebots-archive/'
```

### Check before uploading anything

```bash
timeout 60 venv/bin/python training/gpu_queue.py status
```

An upload is not a GPU job and does not need the queue, but a 4 GB rsync during a training run
evicts its page cache and spikes epoch time about 30x until it recovers. Upload while the
queue is empty, or accept that you just slowed someone else's arm down.

## Step 2: damage as a random variable

New module `training/synthetic/synthgen/damage.py`, new `[damage]` block in `config.toml`,
applied per robot instance after load and before the segmentation pass, so bboxes and keypoint
visibility are computed on the damaged silhouette with no annotation changes.

### Two mechanisms, chosen by mesh structure

**Part removal (CAD robots).** `import_gltf_as_robot` returns the GLB's mesh objects as a
list, so our robots arrive already split into parts. Delete a random subset.

- `severity ~ U(0.05, 0.30)` as a fraction of removable parts.
- Protect the chassis and any part that anchors a keypoint. `[robots.keypoints]` front and
  back are model-frame offsets, so deleting the part under one leaves the keypoint floating in
  air and the label becomes a lie. Protected set is a name-pattern list per robot plus a
  geometric fallback: any part whose bounding box contains a keypoint.
- Protect parts above a volume fraction so a single delete cannot remove most of the robot.

**Chunk removal (Meshy opponents).** Those GLBs are usually one fused textured mesh, so part
removal does nothing.

- First try `separate loose parts`. If it yields more than one island, fall back to part
  removal above.
- Otherwise apply a boolean difference with a randomly placed cutter (cube or icosphere)
  seeded on the mesh surface, scaled to remove `U(0.03, 0.20)` of the bounding volume.
- Reject and resample if the cut leaves the mesh non-manifold in a way that breaks the
  segmentation pass, or removes a keypoint anchor region.

Cosmetic-only damage (scorch marks, roughness patches, darkened albedo) is a third mechanism
and is out of scope for the first pass. Note it as a follow-up.

### Sampling and bookkeeping

- Per instance: `p_damage = 0.35`. Roughly a third of robots in a frame are damaged, which is
  about what a late-round fight looks like.
- Write `manifest.jsonl` beside `images/` and `labels/`, one row per frame:
  `{"image": "000123.jpg", "venue": "nhrl", "mount": {...}, "instances": [{"class": "mrs_buff_mk3", "damage": 0.18, "mechanism": "parts"}]}`.

The manifest is what makes question 4 free. Damage-off arms filter to frames where every
instance has `damage == 0`, damage-on arms take everything. At `p_damage = 0.35` and one to
three robots per frame, roughly 40 to 50 percent of frames are fully undamaged, so a 20k
render yields an 8k to 10k clean pool. Confirm that split on the probe and raise the render
count if the clean pool comes out too thin to match the damage-on arm.

Sanity gate before the full render: render 200 damaged frames and page through
`sheet.png`. Reject the mechanism if robots come out unrecognizable rather than chewed.

**Correction, 2026-09-12.** `render_scenes.py` writes no `sheet.png`; that comes from
`render_cage_samples.py`. Page the output with `training/yolo/validate_yolo_dataset.py
<dataset>`, which draws boxes and keypoints on a grid, and crop in on the largest damaged
instance from `manifest.jsonl` before grading. At the cage mount our robots run 34 to 70 px,
where damage is not visible at all.

## Step 3: render

Two datasets, flat, no split. Splits are image lists later.

```
training/data/synth_cage_nhrl_<date>/{images,labels,manifest.jsonl,data.yml}
training/data/synth_cage_massd_<date>/{images,labels,manifest.jsonl,data.yml}
```

`[cage].probability = 1.0` and `[cage].spec` go in per-venue copies of `config.toml`
(`config_cage_nhrl.toml`, `config_cage_massd.toml`) rather than being passed on the command
line, so the render is reproducible from a file.

### Shard across the three A6000s

The queue is strictly serial: one job at a time, whatever `-d` says. So three shards submitted
as three jobs would run one after another. To use all three GPUs the render is **one** queue
job that launches three containers and waits.

Two small changes make that work:

1. `run_synthetic.sh` hardcodes `--gpus all` and forwards no CUDA env. Add a
   `CUDA_VISIBLE_DEVICES` passthrough to `docker_env_args` so a shard can be pinned to one
   GPU while the container still sees all three devices.
2. New `training/synthetic/docker/render_shards.sh <config> <out> <total> <shards>`: launches
   one container per shard with disjoint `--start-index` and distinct `--seed`, each writing
   to its own `<out>_shard<i>`, waits on all of them, then hardlink-merges the shards into one
   flat `<out>`. Use `os.link`, not a forking `cp` loop, which is pathologically slow at this
   scale.

Separate shard directories rather than one shared `--out`: `--start-index` keeps image
filenames disjoint, but `data.yml`, `sheet.png` and `manifest.jsonl` are written per run and
would race.

```bash
ssh megamind
cd /home/ben/auto-battlebot
venv/bin/python training/gpu_queue.py submit --name render_cage_nhrl --by <agent> -d 0 1 2 -- \
  bash training/synthetic/docker/render_shards.sh \
    config_cage_nhrl.toml ../data/synth_cage_nhrl_<date> 20000 3

venv/bin/python training/gpu_queue.py status
venv/bin/python training/gpu_queue.py logs -f
```

MassD is the same command against `config_cage_massd.toml`, seed base 200, once the other
agent's integration lands. Submit NHRL first so the render starts while MassD is still landing.

Check `status` before submitting. A render that owns all three GPUs for many hours pushes every
queued training arm back by that much, so submit it with a name that says what it is and tell
whoever else is queued.

Budget: about 200 KB per 1280x720 JPEG, so 40,000 frames is roughly 8 GB. megamind's `/` has
76 GB free. Fine, and step 1 covers moving the finished datasets to `/media/storage`.

Gates after each render:

```bash
ssh megamind 'cd /home/ben/auto-battlebot && venv/bin/python \
  training/yolo/validate_yolo_integrity.py training/data/synth_cage_nhrl_<date> --strict'
```

- Zero errors, zero warnings.
- Per-class counts printed and recorded. Our robots must not be rare.
- Keypoint visibility distribution: how many rows carry vis-0 keypoints. A jump against the
  randomized pool means the mount or the glass is eating keypoints.
- Drop rate from `min_robot_visibility`. If more than 25 percent of scenes are discarded, the
  mat margin or the distractor count needs a look before burning the rest of the budget.
- Eyeball `sheet.png` and 50 random frames.

## Step 4: arms

Every arm is a `.txt` image list, built by extending `make_scaling_splits.py` to draw from
multiple source datasets with per-source counts. Frames are drawn by a single fixed shuffle
per source so arms nest: the 10k domain arm is a prefix of the 20k one, and a drop in accuracy
cannot be blamed on which frames got picked.

Constants across arms: `yolo26x-pose`, imgsz 640, batch and epochs fixed, seed 0, 3x A6000 DDP
through the queue, `--save-period 25`.

`R` = randomized frames from `training/data/synthetic`. `D` = domain frames, split evenly
between the two venues unless noted. The 452 real frames are in every arm.

| Arm | R | D | Answers |
| --- | --- | --- | --- |
| `base` | 17,995 | 0 | baseline, the corpus today |
| `d2500` | 17,995 | 2,500 | Q1 |
| `d5000` | 17,995 | 5,000 | Q1 |
| `d10000` | 17,995 | 10,000 | Q1 |
| `d20000` | 17,995 | 20,000 | Q1 |
| `d40000` | 17,995 | 40,000 | Q1, the whole render |
| `swap_half` | 10,000 | 10,000 | Q2, total synthetic held at 20,000 |
| `swap_all` | 0 | 20,000 | Q2 and Q3 |
| `nhrl_only` | 0 | 20,000 NHRL | Q2, venue transfer |
| `massd_only` | 0 | 20,000 MassD | Q2, venue transfer |
| `nodamage` | best mix | same count, damage-free frames only | Q4 |

`base`, `swap_all` at 20k and `d20000` share three points on the amount curve, so the grid is
eleven arms, not sixteen.

Eleven `yolo26x-pose` runs is a lot of queue time. Run the grid on `yolo26s-pose` first to
shape the curves, then confirm the three or four arms that matter on `yolo26x-pose`. That is
what `model_size` and `meshy_grade` did, and it is the difference between a week and a month.

### The step-count confound

`d40000` sees 3.2 times the frames of `base`, so at fixed epochs it also gets 3.2 times the
gradient steps and part of any win is just more training. Handle it the way
`synthetic_arms_2026-07-31` did: keep epochs fixed at 100 for the headline table, and use the
`--save-period 25` checkpoints to read every arm again at matched frame-presentations. Report
both. If the win survives at matched steps it is the data.

### The val set is not a decision surface

`all_robot_keypoints/val` is 2,004 synthetic and 45 real. Arms trained on more synthetic will
look better on it for reasons that have nothing to do with the field. Use it for training
bookkeeping and early-stopping only. Every claim in the writeup comes from `score.py` on
`nhrl_keypoints_eval_test`.

## Step 5: score

```bash
venv/bin/python training/yolo/convert_to_onnx.py data/models/yolo26x-pose_<arm>_<date>.pt
venv/bin/python training/yolo/convert_to_tensorrt.py data/models/yolo26x-pose_<arm>_<date>.onnx --workspace 4

venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate base=data/models/yolo26x-pose_base_<date>_x86_64_sm89.engine \
  --candidate d20000=data/models/yolo26x-pose_d20000_<date>_x86_64_sm89.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot" \
  --taxonomy training/model_eval/taxonomy.yaml --conf 0.5 --baseline base \
  --output training/data/nhrl_keypoints_eval_test/scores_domain_mix
```

Build engines on pathfinder (sm89), not megamind (sm86). Check the printed
`num_keypoints=2 num_classes=4` line on every run: a wrong `--labels` length misparses the
tensor and returns near-zero recall that looks like a broken engine.

Score three ways:

1. **Pooled**, all 688 frames, paired bootstrap against `base`.
2. **Per venue.** NHRL May (590 frames) against MassD Aug (98). This is the direct read on
   question 2: NHRL-cage synthetic should move the May recordings and MassD synthetic should
   move the August one. If `nhrl_only` lifts MassD as much as `massd_only` does, then the win
   is generic cage-ness, not venue match, and there is no reason to build a scene per venue.
3. **Per recording.** Each recording is one opponent, so per-recording recall is per-opponent
   grade. Pooled AP understates the good cases.

Keypoint metrics go through `taxonomy_keypoint_ours.yaml`, which excludes opponents so heading
error reflects our robot.

### Pre-registered criteria

Write these down before the first score run and do not move them afterwards.

- **Adopt** a domain mix if agnostic opponent recall on the pooled eval rises by at least 0.03
  with a 95 percent CI excluding zero, and our-robot heading error does not get worse by more
  than 1 degree.
- **Drop randomized** if `swap_all` is within 0.01 recall of `d20000` on both venues.
- **Damage helps** if `damage-on` beats `nodamage` on opponent recall with a CI excluding zero.
- Anything else that moves is an unregistered finding and needs a confirmatory run before it
  drives a deployment decision. `synthetic_arms_2026-07-31` pre-registered recall, got a
  precision win, and had to label it unregistered. Same discipline here.

## Step 6: grow the eval set with pre-labels

The 98 MassD frames are the weak point. Any per-venue claim about MassD rests on them, and 98
frames gives a wide CI no amount of bootstrap resampling fixes. The fix is more labeled
frames, and the pre-label loop is what makes that affordable.

**Assumption to confirm:** `nhrl_keypoints_eval_test` is already fully labeled (688 `pass`
frames, no empty label files), so "hand label the NHRL eval test set" means growing it, mostly
on the MassD side, plus finishing `nhrl_cage_high_eval` (650 frames, 201 still empty). Both
use the identical loop below. Say which one comes first and I will order the steps.

### The loop

1. **Sample frames.** `make_eval_dataset.py` at a higher `--per-video` over the MCAPs, writing
   empty labels.

```bash
venv/bin/python training/model_eval/make_eval_dataset.py \
  'data/saved_recordings/MassD_2026-08-29/*.mcap' \
  --output-dir training/data/nhrl_keypoints_eval_test --per-video 250 \
  --extra-classes opponent house_bot
```

2. **Pre-label.** New script `training/model_eval/prelabel_dataset.py`: runs a `.pt` over each
   subdataset's `images/` and writes YOLO pose rows into `labels/`. `export_labels.py` cannot
   do this, since it reads detection topics out of a `label_playback` MCAP and these frames
   have none.

   Run it at `--conf 0.15`, not 0.5. Deleting a spurious box in `edit_labels.py` is one
   keypress; drawing a missing box plus two keypoints is a dozen actions. Bias the pre-labeler
   toward over-detection.

3. **Correct** in `edit_labels.py`, `space` to mark reviewed and jump.

4. **Merge** the review state with `merge_validation_state.py`.

5. **Second round.** The first pre-labeler is whichever arm is best today. Once `d20000` or
   `swap_all` exists, re-pre-label the frames not yet reviewed with it. A model trained on
   MassD-domain synthetic should pre-label MassD frames better than anything trained without
   it, which is the practical payoff of this experiment independent of the deployment result.

### Two guards, because pre-labeled GT can poison the eval

- **Miss bias.** A frame where the pre-labeler sees nothing arrives empty, and an empty frame
  looks reviewed at a glance. Every frame gets opened at fixed zoom, and no frame is marked
  reviewed from the thumbnail.
- **Blind audit.** Hold out 10 percent of frames, label them from empty with no pre-labels,
  and compare box counts against the pre-labeled population. That number is the measured bias
  the eval carries, and it goes in the writeup. If the pre-labeled set has systematically
  fewer boxes, the eval flatters every model, including the one that did the pre-labeling.
- **Never pre-label with an arm and then score that arm as the headline** without the audit
  number beside it.

Also measure the speedup: time 50 frames pre-labeled and 50 from empty. If pre-labeling does
not actually save time, say so and drop it.

## Risks

- **Render throughput is unmeasured.** Everything downstream is scheduled off step 0b. At 10
  seconds per frame, 40,000 frames is 111 GPU-hours, which is 37 wall-clock hours sharded
  three ways. That is 37 hours of no training for anyone. If the probe lands near that, cut
  the render to 10,000 per venue and spend the saved time on the amount curve instead of its
  tail.
- **The render blocks the queue.** Rendering on the training box is the whole point of step 1,
  and the cost is that other agents' arms wait. Announce the submission, and do not start the
  MassD render until the NHRL dataset has passed its gates, so a bad spec does not cost two
  slots.
- **megamind `/` is at 92 percent.** 4.1 GB of assets, 11 GB of docker image and 8 GB of
  render output fit in the 76 GB free, but nothing else large does. Move each finished dataset
  to `/media/storage` before starting the next render.
- **Docker mounts only the repo root.** Assets symlinked out to `/media/storage` dangle inside
  the container. Keep render inputs and outputs as real directories under the repo on
  megamind.
- **Pruned assets are a bet on two code paths.** Skipping the HDRIs and 511 of 524 texture sets
  rests on `[cage].probability = 1.0` never taking the arena path and on `load_ccmaterials`
  honoring `used_assets`. The 200-frame probe is what confirms it. If the probe logs missing
  textures or falls back to a default world, upload the rest before committing to 40,000
  frames.
- **Intrinsics mismatch.** Covered by 0a, and it is the single item most likely to make the
  domain arms underperform for a reason unrelated to the hypothesis.
- **MassD integration is in flight.** The MassD render cannot start until the other agent's
  generic-pipeline work lands. Start the NHRL render first.
- **Damage can make labels wrong.** A keypoint on a deleted part is a false label that trains
  the heading head toward noise. The protected-part rule and the 200-frame visual gate are the
  defense; if either is shaky, ship damage as a separate small dataset instead of mixing it
  into the main render.
- **Eleven arms of `yolo26x-pose`** will not fit a reasonable week. The `yolo26s-pose` shaping
  pass is not optional.
- **Segmentation stops at glass.** Any mount rendering through polycarbonate loses its labels.
  This is no longer handled by keeping the camera inside: every marked mount sits outside, and
  `apply_one_way_glass` hides the pane a camera looks through from outside, per frame. That is
  why `inset_m` stays strictly negative, so the hiding always applies. The 2026-09-12 probe
  found keypoint visibility unchanged against the randomized pool, 3.9 percent flag-0 against
  5.2, so the mechanism holds.

## Next steps

Items 1, 2, 3 and 5 are done; see the status section. What is left:

1. Grade the 100-frame inspection renders on megamind (jobs 42 and 43). Reject or keep the
   cutter mechanism, given the white interior faces it exposes.
2. Run the timing probes at 128 and 64 samples on both specs, on one A6000. Overriding samples
   means a variant config, since `--render-samples` does not reach a cage scene. Confirm the
   log says 13 ground textures rather than failing, and record seconds per frame, VRAM, and
   drop rate.
3. Settle `imgsz` for step 4. The domain frames run our robots at roughly half the pixels the
   randomized pool does, and 640 is what starved the cage-high detector.
4. Add the `CUDA_VISIBLE_DEVICES` passthrough to `run_synthetic.sh` and write
   `docker/render_shards.sh`.
5. Submit the NHRL 20k render to the queue. Gate it, allowing the `house_bot` warning on the
   MassD half, move it to `/media/storage`, then submit MassD.
6. Extend `make_scaling_splits.py` to multi-source counts, build the eleven arm lists, and
   submit the `yolo26s-pose` shaping grid to `gpu_queue.py`.
