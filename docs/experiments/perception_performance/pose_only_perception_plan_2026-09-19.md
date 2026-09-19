# Pose-only perception: one `yolo26x-pose` model, no bounding box model

Plan for replacing the two-model perception batch (`yolo26s` detector plus `yolo26s-pose`
keypoints) with a single 4-class `yolo26x-pose` model, on the fixed ZED One S with the homography
field. The robot blob model becomes a noop. A motion detector fills the frames the pose model does
not reach, and gets its own plan once this one has a measured YOLO rate to design against.

Nothing here has run. The numbers quoted are from earlier reports, cited inline.

Updated 2026-09-19: the `meatball_basement` scene is built, so the dataset gains a 10,000-frame
basement render (step 1a) and the arms gain a control for it.

## What changed since the last recommendation

Three decisions, all made 2026-09-19:

1. **The camera is a fixed ZED One S and depth is gone.** Position and heading both come from
   pixels through the field homography. The camera does not move during a match, which is what
   makes a motion detector possible (`stationary_bgsub_methods_2026-09-07.md`: within one fight
   the cage camera moves at most 3.25 px, and 18 of 20 clips stay under 1 px).
2. **YOLO runs as fast as it can and a motion detector fills the other frames**, so all 60 fps
   reach the filter. The pose model no longer has to fit one frame period. It has to refresh
   identity and heading often enough that the filter can coast between refreshes.
3. **The Kalman filter takes position or position plus heading.** For our robot,
   `KalmanMotionEstimator::update` runs a 3-row `ekf_update<5,3>` on x, y, theta when the
   measurement carries keypoints and a 2-row position update when it does not
   (`kalman_motion_estimator.cpp:288-374`). Opponents are position-only in every case
   (`ekf_update<4,2>`, `:454-463`), and `measurement_noise_for` (`:209-222`) picks
   `keypoint_position_sigma_m` or `blob_position_sigma_m` by whether keypoints are present. So a
   headingless motion measurement already has a path into the filter with its own noise.

Decisions 2 and 3 are what make `x` worth another look. `jetson_model_size_latency_2026-09-19.md`
measured the detector `x` at 33.1 ms of solo GPU time at 640x640 on the Orin NX and estimated
49.8 ms in-batch beside the keypoint model, which is why it was unaffordable. With the detector
gone there is no batch partner to contend with, and the report's own estimate for `x` at 384x640 is
near 21 ms solo. That number is scaled from the detector, never measured, and never measured for
the pose head. Step 0 measures it.

## Why one model can do both jobs

The domain-mix arms already train four classes (`mr_stabs_mk2`, `mrs_buff_mk3`, `nhrl_robot`,
`house_bot`), each with a front and a back keypoint, and the C++ side already routes them:
`OPPONENT_FRONT/BACK` and `HOUSE_BOT_FRONT/BACK` are in `[keypoint_model.label_map]`, and
`[robot_filter.label_mapping]` sends `OPPONENT` to `THEIR_ROBOT_1..3` and `HOUSE_BOT` to
`NEUTRAL_ROBOT_1` (`config/_common.toml`). An opponent keypoint pair becomes a `THEIR_ROBOT` track
with a heading today (`robot_front_back_filter.cpp:449-497`). Blob measurements are forced to
identity rotation (`:434`), so the pose model gives the opponent track something the detector
never did.

What `x_d40000_ep50` scored (`synthetic_domain_mix_2026-09-18.md`, conf 0.5):

| Eval | Opponent recall | Opponent precision | Our recall | Our kp err | Our heading err |
| --- | --- | --- | --- | --- | --- |
| ZED, 688 frames | 0.419 | 0.967 | 0.887 | 5.81 px | 5.28 deg |
| Cage-high, 636 frames | 0.910 | 0.917 | 0.954 | 1.05 px (self-seeded GT) | 0.56 deg (self-seeded GT) |

**The opponent recall on ZED footage is the weak point of this whole plan.** The `yolo26s`
detector this replaces scored 0.839 agnostic recall on the same 688 frames
(`model_size_2026-09-04.md`), though under `taxonomy_merged.yaml` with the house bot pooled in, so
the two numbers are not comparable as written. Nobody has scored the detector and a pose arm under
one taxonomy. That comparison is gate A below, and it runs before any C++ work.

The cage-high row is the friendlier one and the closer match to a fixed mount, but its ground
truth was pre-labelled by this same arm, and it goes into training under this plan, so neither row
survives as an eval. Step 3 builds the replacement.

## Questions and gates

Written before anything runs. Do not move them afterwards.

- **Gate A, opponent detection.** Score the deployed `yolo26s` detector and the pose arm on the
  new eval under `taxonomy_opponent.yaml`, conf picked per model at its own F1 peak on a held-out
  half of the frames. Pose-only goes ahead if the pose arm's opponent recall is within 0.05 of the
  detector's, or if its F1 is higher. The 0.05 is the noise floor `base` showed across its own
  checkpoints (0.284 to 0.367). If the gate fails, the fallback is the pose model for our robot
  plus the motion detector for opponent position, and the detector still goes.
- **Gate B, latency.** `yolo26x-pose` at the deployed input shape runs under 33.3 ms per
  inference on the Orin NX, measured in the C++ application with no second model loaded, so
  heading refreshes at 30 Hz or better. If it lands between 33.3 and 50 ms, measure what 20 Hz of
  heading costs in the sim sweep before rejecting it. Above 50 ms, train `yolo26l-pose` and
  `yolo26m-pose` on the same data instead.
- **Gate C, keypoints.** Our-robot heading error on the new eval is no worse than the deployed
  `yolo26s-pose` by more than 1 degree, with the CI read against zero.
- Anything else that moves is unregistered and needs a confirmatory run.

## Step 0: time `yolo26x-pose` on the Orin NX

One afternoon, no training, and it can kill the plan early.

```bash
# megamind: rectangular ONNX from the existing x arm. -o keeps the square export in place.
venv/bin/python training/yolo/convert_to_onnx.py \
  data/models/yolo26x-pose_d40000_2026-09-16_last.pt --imgsz 384 640 \
  -o data/models/yolo26x-pose_d40000_2026-09-16_last_rect384x640.onnx

# Jetson: build, pin clocks, idle the box, then time it the way the 2026-09-19 report did.
venv/bin/python training/yolo/convert_to_tensorrt.py \
  data/models/yolo26x-pose_d40000_2026-09-16_last_rect384x640.onnx --workspace 2
sudo jetson_clocks
venv/bin/python training/model_eval/benchmark_engines.py \
  --candidate x384=data/models/yolo26x-pose_d40000_2026-09-16_last_rect384x640_aarch64_sm87.engine \
  --frame data/bench_frame.png --iterations 300 --csv out.csv
```

Also build and time `416x640`, which is the letterbox shape for the 16:10 1920x1200 sensor mode,
and one larger shape (`768x1280` or `800x1280`). `rgb_camera_migration_2026-09-09.md` puts a 20 cm
robot at 52 to 110 px across a 1920-wide frame from a 1.2 m mount, so a 640-wide tensor shows the
network 17 to 37 px robots and that report calls `imgsz 1280` the floor. `x` at 1280 will not pass
gate B on this box. The two ways out are a smaller model at 1280 or `x` on a native-resolution
crop around each track (step 5f), and step 0's table is what chooses between them.

The solo benchmark is the honest number here in a way it was not for the two-model rig, because the
1.5x in-batch factor came from two engines sharing the GPU and this plan has one.

## Step 1a: render 10,000 basement frames

The `meatball_basement` scene landed on 2026-09-19 (`playground/basement_scene/README.md`): the
1.52 m plywood drive-test box in its corner of stone walls, fitted from one frame of our own ZED on
a stand, with the floor grading at 0.973 SSIM against the target. `config_cage_meatball.toml` pins
a render to it. It is the third domain venue, and it is a different kind of venue from the other
two for three reasons:

- **It is the scene the real training frames came from.** `pose_model_size_corpus_2026-09-07.md`
  describes the corpus's real frames as `mrs_buff_mk3` sessions in a plywood test box. Until now
  the render and the real frames shared no scene, so nothing tied a rendered Mrs Buff to a
  photographed one against the same floor. This render does.
- **The mount bracket is low and close.** 0.40 to 0.95 m above the floor and 0.30 to 0.80 m outside
  the rail, against cage mounts for the other two. That is nearer the 1.2 m ZED One S mount than
  either a robot-height view or an overhead broadcast camera.
- **It has no house bot** (`[house_bot_box] enabled = false`), so it adds nothing to the class the
  domain arms are weakest on.

Render it the way the two 20,000-frame venues were rendered, a third per view, so the basement
frames differ from the rest of the domain pool by venue and not by lens:

```bash
venv/bin/python training/gpu_queue.py status   # the GPUs were held by a vLLM job on 2026-09-19
venv/bin/python training/gpu_queue.py submit --name render_cage_basement_10k --by <agent> -d 0 1 2 -- \
  venv/bin/python training/synthetic/render_shards.py config_cage_meatball.toml \
    --out ../data/synth_cage_basement_<date> --total 10000 --gpus 0 1 2 \
    --views distorted pinhole rectified --seed-base 400
```

`--seed-base 400` keeps its seeds clear of NHRL's (0) and MassD's (200), and the view order puts
the run that lands one frame short on a third view. The earlier renders ran 8.9 to 9.3 s per
warped frame per GPU and took 11.3 h and 13.0 h for 20,000 frames, so budget about 6 h of queue.
Nothing under `training/synthetic` changes while it runs, since the container mounts the repo
live.

Before the full job, a 200-frame smoke render through the same command with `--total 200`, read by
eye and by the gate report. This scene has features no other venue exercises (`[[walls]]`,
`[[blocks]]`, a panorama world), and the NHRL render's floating-robot bug was only caught by
looking at frames:

```bash
venv/bin/python training/synthetic/domain_render_report.py training/data/synth_cage_basement_<date>
```

Same gates as the first two renders: zero integrity errors, the clean share and hidden-keypoint
share inside the bands those renders passed at (59.1 to 62.6 percent clean, 2.4 to 2.9 percent
hidden), and robot box sizes reported beside the other venues'. The box is 1.52 m against 2.35 m
cage floors and the camera is closer, so expect larger robots than the 34 to 70 px the cage render
gave. Write the measured range into this plan, because it decides whether these frames help the
small-robot case or only the near one.

Move the finished dataset to `/media/storage` if `/` is tight; it had 261 GB free on 2026-09-19.

## Step 1b: dataset

Full `d40000` composition plus the basement render and the cage-high frames:

| Source | Frames |
| --- | --- |
| Randomized synthetic (`training/data/synthetic` via `all_robot_keypoints`) | 17,995 |
| Domain render, `nhrl_cage` and `massd_arena`, a third per view, damage on | 40,000 |
| Domain render, `meatball_basement`, a third per view (step 1a) | 10,000 |
| Real, from `all_robot_keypoints` | 452 |
| Cage-high, `training/data/cage_high_x50_conf044`, `pass` frames only | 636 |
| Total | 69,083 |

Reasons, all from `synthetic_domain_mix_2026-09-18.md`:

- On fixed-mount footage the domain-heavy arms lead: `d40000_real3x` 0.911 and `d40000` 0.870
  opponent F1 against `swap_half` 0.807, and `d40000` holds 0.988 precision on our robot at
  1.68 degrees of heading error. Venue renders transfer to the viewpoint they were rendered from,
  and the render's mounts are cage mounts.
- Once a motion stage bridges short dropouts, a miss costs a few frames of coasting and a false
  lock costs a track the motion stage will then follow. That moves the trade toward precision,
  which is the axis domain count buys.
- Keep the randomized pool: `swap_all` is the one arm whose keypoints go backwards (5.69 px
  against 3.69 px on cage-high).
- Keep both venues: `nhrl_only` and `massd_only` lose keypoint precision (7.37 and 11.66 px
  against `base` 4.78 px).
- Write the real frames once. `real3x` recovered nothing on ZED footage.
- A third venue is the direction the venue arms point. `nhrl_only` and `massd_only` both lost to
  the two-venue arms on ZED footage, and the reading was that one venue is a narrower appearance
  distribution than the randomized pool it replaced. The basement is 10,000 frames against 20,000
  for each cage because it is not a venue we fight in. Its job is to tie the render to the real
  frames and to add a low mount, and the `s` control in step 2 measures whether it does either.

`make_domain_mix_arms.py` needs two changes:

1. **A second real source.** It draws real frames from the corpus alone. Add one with a
   `validation_state.json` filter, selecting `pass` and never `.edit_state.json`. The 14 frames in
   `validation_backup/` stay out.
2. **Venue pinning on the existing arms.** `domain_order` interleaves every venue it is handed so
   any prefix splits evenly between them. Passing the basement render as a third `--domain` would
   therefore put basement frames into `d2500` through `d40000` and move every list the grid
   trained on. Pin the existing arms to `venues=(NHRL, MASSD)`, which is what they drew from, and
   give the new arms all three. With unequal venue sizes an even interleave runs out of basement
   frames at 30,000, so the new arms take every frame of every venue and are not prefixes of
   anything.

Rebuild every existing arm into a scratch directory afterwards and confirm the eighteen `.txt`
lists still reproduce byte for byte, as the `nodamage_swap_half` addition did.

Build `swap_half_cagehigh` from the same change. It is the control for what the 636 frames buy and
it trains on `s` in 2 h 20 min.

The render used e-CAM25 intrinsics. The view arms showed detection tolerates the lens
(`view_distorted` ties `view_pinhole` on MassD at 0.571 recall), so the 40,000 frames are reusable
now. Re-render a slice on the ZED One S intrinsics and mount once both are calibrated, and treat it
as a new arm.

## Step 2: train and export

| Arm | Model | Frames | Epochs | Presentations | Purpose |
| --- | --- | --- | --- | --- | --- |
| `x_d50000_cagehigh` | `yolo26x-pose` | 69,083 | 30 | 2.07 M | The candidate |
| `s_d50000_cagehigh` | `yolo26s-pose` | 69,083 | 30 | 2.07 M | Size control, and the fallback if gate B fails |
| `s_d40000_cagehigh` | `yolo26s-pose` | 59,083 | 35 | 2.07 M | Basement control: the same mix without step 1a's frames |
| `s_swap_half_cagehigh` | `yolo26s-pose` | 21,088 | 100 | 2.11 M | Mix control |
| `s_swap_half` | `yolo26s-pose` | 20,452 | 100 | 2.05 M | Already trained, the cage-high control |

`d50000` is the 40,000 cage frames plus the 10,000 basement frames. Every arm lands within 3
percent of the same presentation count, so the three controls each change one thing against the
`s` candidate: model size, the basement frames, and the domain share.

Epochs come from the presentation rule: opponent recall peaked near 2 M frame-presentations on
every mix and fell past 3 M (`d40000` 0.606 at epoch 50, 0.295 at epoch 100). Everything else is
the grid's constants: `imgsz 640`, `-b 96`, 3-GPU DDP through the queue, `--seed 0`, `--cache ram`,
pretrained start. `run_domain_mix_arm.sh` hardcodes `SAVE_PERIOD=25`, which gives a 30-epoch arm
one intermediate checkpoint; make it an argument and use 10, so checkpoints land at 0.69, 1.38 and
2.07 M. Ship `last.pt`. Val is 2,004 synthetic frames and 45 real ones and selects for the
renderer.

`--cache ram` held about 49 GB per DDP rank on `d40000`'s 58,447 frames, with 144 GB of megamind's
251 GB in use. At 69,083 frames that scales to about 58 GB per rank and 170 GB in total, which
fits with less room than before. Check `free -g` once caching finishes on the first `d50000` arm
and stop background watchers first, as the `d40000` run had to.

```bash
venv/bin/python training/gpu_queue.py submit --name po_x_d50000_cagehigh --by <agent> -d 0 1 2 \
  --work 2072490 --profile yolo26x-pose@640 -- \
  bash training/yolo/run_domain_mix_arm.sh training/data/domain_mix_arms_<date> d50000_cagehigh yolo26x-pose 30
```

`dm_x_d40000_ep50` took 12 h 18 min for 2.92 M presentations, so expect about 8 h 45 min for the
`x` arm, and `dm_s_d40000_ep50` took 3 h 16 min, so about 2 h 20 min for each `s` arm. The queue
has both histories under their `@640` profiles.

`run_domain_mix_arm.sh` builds the square 640 engine. Add the rectangular export beside it, at
whichever shape step 0 picked. FP16 only: `int8_quantization_2026-09-06.md` measured INT8 at
0.032 recall below FP16 on `s` and found `x` lost its whole recall gain.

If step 0 points at native-resolution crops, this step gains a crop-trained arm: cut training
frames with `crop_yolo_dataset.py` so robots reach the tensor at 52 to 110 px, which is the scale
the deployed crop would show.

## Step 3: a new eval set

Both existing sets stop being evals under this plan. Cage-high goes into training. The ZED set
stays clean of training frames, but its NHRL recordings are from 2026-05-01 and 05-02 and the
cage-high NHRL recordings are `nhrl_may26` Mrs Buff fights, which look like the same event and
possibly the same fights from another camera. Opponent recall on it is contaminated by appearance
once cage-high is trained on. Keep scoring it as a secondary read and say so beside the number.

Until the ZED One S records a fight:

- Source: `data/downloads/brettzone_cage_high`, 20 fights at 1080p. Exclude the six fights in the
  training cage-high set and any fight whose robots appear in it.
- Pre-label with `x_d40000_ep50` at conf 0.15 through `prelabel_dataset.py`, with a seeded 10
  percent hold-out labelled from empty. The cage-high set skipped the hold-out
  (`--holdout 0`), and that is why its keypoint numbers cannot be quoted.
- The pre-labeller is a parent of the candidate. Report the hold-out's box-count bias beside every
  headline number, per the plan's guard in `synthetic_domain_mix_plan_2026-09-12.md`.
- Most of those fights have no Mrs Buff in them. They score opponent and house bot detection
  (gate A) and not our keypoints (gate C). Gate C needs Mrs Buff footage from a fixed mount, which
  today means the MassD broadcast clips not already used, or the first ZED One S recordings.
- The basement SVOs are a third source for gate C: Mrs Buff, a fixed ZED on a stand, 1280x720. Two
  checks before using one. The corpus's real frames are plywood-box sessions, so list which
  recordings they came from and take eval frames only from recordings that gave none. And the
  basement render is fitted to frame 4560 of `2026-04-19T17-01-18`, so an eval drawn from that
  session flatters the `d50000` arms against the basement control. Use a different session, and
  prefer the August rebuilt box, which the render does not model.

Replace this set with ZED One S footage at the real mount as soon as the camera records, labelled
in the image form the pipeline feeds the model.

## Step 4: score

Same tooling as the domain-mix grid: `score.py`, 1000-sample paired bootstrap, `--labels
mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot`, and check the printed `num_keypoints=2
num_classes=4` line on every run.

1. Gate A: detector against pose arms, `taxonomy_opponent.yaml`.
2. Gate C: `taxonomy_keypoint_ours.yaml`, against the deployed `yolo26s-pose`.
3. A confidence sweep per arm at 0.15, 0.25, 0.35, 0.5. On ZED footage every arm's F1 was better
   at 0.25 than at 0.5 and on cage-mount footage sixteen of eighteen peaked at 0.5, so the
   threshold belongs to the camera. Pick it on half the eval frames and report on the other half.
4. Wrong-class rate and extra boxes per class, as the cage-high per-class table did. With one
   model doing identity for every robot, an opponent named `mrs_buff_mk3` moves our own EKF, which
   is worse than any miss.
5. House bot recall on its own line. The best arm found 300 of 512, and the house bot keep-out is
   built from that track.
6. `s_d50000_cagehigh` against `s_d40000_cagehigh`, bootstrapped, on every eval. The basement
   frames stay in the candidate's mix if they do not cost opponent recall or our-robot heading
   error with a CI excluding zero. They are expected to pay on basement footage and on low
   mounts; a gain confined to the basement eval is a reason to keep them for drive testing and
   says nothing about a cage. If they cost recall anywhere, `x` retrains on `d40000_cagehigh`.

## Step 5: C++ rework

The pipeline already runs pose-only if the config says so: `NoopRobotBlobModel` is registered,
`RobotFrontBackFilter::correct` skips the blob branch when the blob result is empty
(`robot_front_back_filter.cpp:157`), and the keypoint model reads its input shape from the engine
(`yolo_keypoint_model.cpp:58-64`), so a rectangular `x` engine is a config change. The work below
is what that leaves broken, wasted or misleading. Each item is its own commit with tests.

**5a. Point the static gate at keypoint-model detections.** `runner.cpp:445-447` applies
`StaticDetectionGate` to `robot_blob_keypoints` only, so with a noop blob model it sees nothing.
Without depth the height gate cannot reject either (`reject_enable = false` in
`_video_playback.toml`), so arena-logo false positives reach the filter with no gate at all. 91
percent of MassD false positives sit in fixed field-frame clusters, which is the case this gate
exists for. The gate is label-agnostic on `ModelResultStamped`; the change is wiring plus one
rule: never suppress `OUR_ROBOT` labels, since our own robot sitting still before the match is
the gate's exact definition of a logo. Fix the "robot-blob detections" comments in
`static_gate.hpp:26` and `_common.toml`. Tests: a stationary `OPPONENT` pair is suppressed after
the dwell, a stationary `MRS_BUFF_MK3` pair never is.

**5b. Skip the blob worker when the blob model is a noop.** `ParallelModelBatch` spawns both
workers and waits on both done-ids every frame (`parallel_model_batch.cpp:39-50,105-108`). Set
`parallel_models = false` in the pose-only profiles now. Longer term give
`RobotBlobModelInterface` an `is_noop()` that the runner reads once at startup, so a profile that
forgets the flag does not pay a thread handshake per frame. Keep `ParallelModelBatch`: the motion
stage will want a worker beside the pose model.

**5c. Stop publishing and timing a model that is not there.** `runner.cpp:500` publishes
`last_detections()` to `/blob_detections` every frame and `:417-419` emits a
`robot_mask_model.update` timing of about zero. Gate both on the same `is_noop()`. Downstream,
`export_labels.py` writes an empty `blob/` directory from such a recording; make it say the topic
was empty instead. Confirm the keypoint model's boxes and classes for all four labels reach the
recording, because `export_labels.py` and `compare_cpp_python.py` become the only path from a
match recording to labels.

**5d. Settle where label grouping lives.** `their_robot_labels` and `neutral_robot_labels` are
read only inside `YoloBboxRobotBlobModel`; the keypoint path groups by
`[robot_filter.label_mapping]` through `infer_group_from_frame_ids`
(`robot_front_back_filter.cpp:51`). `config/experiments/mrs_buff_mk3_label_playback.toml` already
flags the split as a foot-gun. Pose-only profiles drop the blob fields by changing the section's
`type`, which the merge handles. Add a parse-time check that every entry in
`keypoint_model.label_indices` has a `label_map` entry and a `label_mapping` entry, so a fourth
class without a frame id fails at startup instead of vanishing in the filter.

**5e. Remove dead keypoint config.** `image_size` is parsed and stored and never read
(`keypoint_model/config.hpp:33`, `yolo_keypoint_model.cpp:16`). Delete it and any profile line
that sets it, since it reads as if it controls the tensor shape and the engine does.

**5f. ROI input for the pose model, only if step 0 asks for it.** `YoloKeypointModel::update`
takes the full frame with `crop=false` (`yolo_keypoint_model.cpp:106`). The crop path is: the
filter's predicted track positions project through the homography to pixel centres, the model
cuts a native-resolution window per track, batches the windows, and maps detections back by the
window offset. It needs a batch-capable engine, a full-frame pass on a fixed cadence and on track
loss so a new robot can enter, and NMS across windows that overlap. This is the largest item in
the plan, which is why it waits on a measurement saying a 640-wide full frame is not enough.

**5g. Keep the blob code; it is where the motion stage lands.** Do not delete
`merge_blob_detections`, `RobotKeypointTracker`, `suppress_blobs_near_our_anchor` or the
`blob_*` and `robot_blob_*` settings. A motion detector produces what the blob model produced:
class-free, headingless positions. It can implement `RobotBlobModelInterface`, enter the filter as
blob keypoints, take `blob_position_sigma_m`, and inherit the rule that a blob near our last pose
is us and not an opponent. Opponent heading then comes from the pose model when it fires and from
the velocity vector otherwise, which `render_opponent` already does
(`kalman_motion_estimator.cpp:224`). The blob-path tests
(`test_robot_front_back_filter.cpp:172-290`, `test_robot_keypoint_tracker.cpp`) stay for the same
reason.

**5h. Decouple the perception rate from the camera rate.** The plan's premise is YOLO at its own
rate under a 60 fps stream. The runner today runs perception once per tick and blocks the tick on
it. Filling frames needs the pose model on a worker that always takes the newest frame, with its
result entering the filter stamped at capture time. `KalmanMotionEstimator` already corrects at
the measurement's own stamp and re-propagates (`kalman_motion_estimator.hpp:28`), so a late pose
result is a case it was built for. This step belongs to the motion-stage plan; it is listed here
because gate B's 33.3 ms only means something once it exists, and because until then a 25 ms pose
model drops the loop to 30 Hz with nothing in between.

**5i. Height above the mat.** The homography maps mat pixels, and the keypoints sit on the robot.
`keypoint_height_meters_per_label` is 0.0 for our robots and 0.12 for the house bot. From a 1.2 m
mount a point 5 cm above the mat lands about 8 cm long at 2 m of range, so check on the ZED One S
calibration target that the projection through `CalibratedFieldFilter` applies these heights, and
measure the real keypoint heights off the CAD instead of leaving them at zero.

## Step 6: playback profile

`config/playback/pose_only_cage_high.toml`, written with this plan. It extends
`playback/brettzone_cage_high` (fixed 1080p view, `VideoPlaybackCamera`, `HomographyFieldFilter`),
sets `[robot_mask_model] type = "NoopRobotBlobModel"`, gives the keypoint model all four labels in
the arms' class order, points it at `yolo26x-pose_d40000_2026-09-16_last`, and turns
`parallel_models` off. `--print-config` resolves it. It has not been run: megamind has no
`data/saved_recordings`, and the only engine that exists is the square sm86 build. The rectangular
and sm89 candidates are listed first so the profile picks them up when step 0 builds them.

```bash
./scripts/build_and_run.sh -c config/playback/pose_only_cage_high.toml
```

What to look for on the first run: three tracks (`OUR_ROBOT_1`, `THEIR_ROBOT_1`,
`NEUTRAL_ROBOT_1`) from one model, an empty `/blob_detections`, and logo false positives arriving
ungated until 5a lands. An SVO twin on `playback/mrs_buff_mk3_playback` is a three-line copy and
worth adding once gate A says the pose model holds up on ZED footage, since that is where its
opponent recall is weakest.

## Risks

- **Opponent recall.** Covered by gate A, and restated because it is the likeliest way this fails.
  `x_d40000_ep50` trails `swap_half` by 0.143 opponent recall on ZED footage at conf 0.5 and by
  0.138 at 0.25, so lowering the threshold does not close it. Its deficit is at NHRL (0.595 against
  0.750) and absent at MassD (0.703 against 0.714).
- **One model, one failure.** Today a keypoint dropout leaves the blob track and the
  `our_keypoint_dropout_blob_*` rule. Pose-only with no motion stage has nothing behind a dropout
  except `our_robot_hold_window_s` of coasting. Do not deploy pose-only ahead of the motion stage.
- **Identity errors move our own EKF.** See step 4 item 4.
- **The latency figure is an estimate of an estimate.** 21 ms is the detector's `x` scaled by the
  geometry ratio measured on `s`. The pose head, the 16:10 shape and JetPack 7 are all unmeasured.
- **No deployment-camera footage exists.** Every eval in this plan is a proxy until the ZED One S
  records a fight. The e-CAM25 that was going to fill this gap was destroyed on 2026-09-19.
- **Single seed.** `data_epoch_min` measured about 0.048 run-to-run recall spread on this corpus.
  A gate A result inside that band needs a second seed before it decides anything.

## Order of work

1. Step 0, the Jetson timing. Gate B can end the `x` branch in an afternoon.
2. Step 3, the new eval set, in parallel. It is hand work and gates everything after it.
3. Gate A on the arms that already exist (`x_d40000_ep50`, `swap_half`, deployed detector), as
   soon as the eval has opponent labels. No training needed to get a first answer.
4. Step 1a, the basement render: a 200-frame smoke render and its gates, then the 10,000 frames,
   about 6 h of queue. It goes on the queue first because every `d50000` arm waits on it, and the
   arm builder changes in step 1b can be written while it runs, since they touch
   `training/yolo` and not `training/synthetic`.
5. Step 1b and step 2: the arm lists and the four new arms, about 16 h of queue. Submit
   `s_d40000_cagehigh` and `s_swap_half_cagehigh` first if the render is still going, since
   neither needs basement frames.
6. Step 4, the full scoring pass.
7. Step 5a to 5e, which are safe to land whatever the gates say, since every one of them is
   correct for a pose-only profile and inert for a two-model one.
8. Step 5f only on step 0's say-so. Steps 5g to 5i move to the motion-stage plan.
