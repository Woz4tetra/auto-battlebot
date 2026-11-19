# Step 0: what `yolo26x-pose` costs on the Orin NX - 2026-09-19

Engine-level latency for the `x_d40000` pose arm at four input shapes, measured on the deployment
Jetson under JetPack 7. This is step 0 of `pose_only_perception_plan_2026-09-19.md`, the one step
that could have killed the plan before any training started. It did not: `yolo26x-pose` at the
deployed 384x640 shape runs at 19.94 ms of raw GPU time, well inside the 33.3 ms frame period.

Predecessor: `jetson_model_size_latency_2026-09-19.md`, which timed the five detector sizes on the
same box with the same tooling. Every comparison below against a detector number comes from there.

## Headline

1. **Gate B passes at the deployed shape, with room.** `yolo26x-pose` at 384x640 costs 19.94 ms
   raw GPU, 20.6 ms with host transfers, and 25.7 ms end to end through the Python path. The gate
   asked for under 33.3 ms. Solo, the model can refresh heading at 48 Hz on GPU time, 39 Hz end to
   end.
2. **The pose head is close to free.** At the same 640x640 geometry, `yolo26x-pose` costs 32.92 ms
   against the `yolo26x` detector's 31.84 ms. Two extra classes, a two-keypoint head, and 3.8 M
   more parameters buy a 1.08 ms bill, 3.4%.
3. **Geometry is what makes `x` affordable, not the model.** The same network costs 19.94 ms at
   384x640 and 74.24 ms at 768x1280, a 3.7x span. The prior report guessed "near 21 ms solo" by
   scaling the 0.64 geometry ratio off `s`. The measurement came in at 19.94 ms.
4. **Of the two ways to reach 1280, the smaller model wins on latency by a wide margin.**
   `yolo26s-pose` at 768x1280 costs 14.43 ms, cheaper than `yolo26x-pose` at 384x640. `x` at
   768x1280 costs 74.24 ms and is not a candidate at any tick rate. Step 5f, cropping native
   resolution windows around each track to feed `x`, is not needed to get a 1280-wide tensor into
   budget. Train an `s-pose` arm instead.
5. **A 16:10 sensor does not break the gate.** The 416x640 letterbox for a 1920x1200 mode costs
   23.59 ms, 18% over 384x640 and still 10 ms inside the period.
6. **Two engines sharing this GPU roughly double each other.** Running today's deployed pair
   concurrently puts the detector at 9.62 ms against 4.61 ms solo and the keypoint model at
   9.84 ms against 4.78 ms. The prior report's fitted 1.5x in-batch factor understates a saturating
   overlap; the GPU serializes, so co-resident work adds.

## Setup

| | |
|---|---|
| Box | Jetson Orin NX 16 GB, `auto-battlebot-compute-1`, JetPack 7.2 / L4T R39.2 |
| Software | TensorRT 10.16.2.10, CUDA 13.2, Python 3.12, `pycuda` built from source |
| Power | MAXN, `jetson_clocks` applied, GPU pinned at 918 MHz, CPU at 1.984 GHz |
| Load | Idle box, load average 0.39 to 0.69 across the runs, nothing else on the GPU |
| Thermals | 62 to 65 C across all zones at the end of the sweep, no throttling |
| Candidate | `yolo26x-pose_d40000_2026-09-16_last`, 62.64 M params, 4 classes, 2 keypoints |
| Controls | `yolo26s-pose_our_robot_keypoints_2026-09-07` (9.75 M params), the deployed `yolo26s` detector and `yolo26s-pose` keypoint model at 384x640 |
| Engines | FP16, `--workspace 2`, built on the Jetson from megamind ONNX |
| Frame | One real 1280x720 frame, 2 labelled robots, the same `data/bench_frame.png` the size sweep used |
| Sampling | trtexec: 300 iterations, 1000 ms warmup. Python: 300 timed iterations, 50 warmup, conf 0.5, NMS IoU 0.45, three passes |

The `x` arm is the checkpoint from `synthetic_domain_mix_2026-09-18.md`. Weights do not change
latency, so the `s-pose` control at 768x1280 stands in for the architecture of a future 4-class
`s-pose` arm. Its head is 12 columns wide instead of 14, which is a postprocess difference only.

## Results - raw GPU compute

`trtexec --loadEngine --noDataTransfers`, kernel time with the input already resident. This is the
floor any runtime can hit.

| engine | shape | median | mean | p99 | qps |
|---|---|---:|---:|---:|---:|
| `yolo26x-pose` | 384x640 | 19.944 ms | 20.263 ms | 20.683 ms | 49.3 |
| `yolo26x-pose` | 416x640 | 23.593 ms | 23.341 ms | 23.639 ms | 42.8 |
| `yolo26x-pose` | 640x640 | 32.916 ms | 33.092 ms | 33.656 ms | 30.2 |
| `yolo26x-pose` | 768x1280 | 74.238 ms | 74.213 ms | 77.963 ms | 13.5 |
| `yolo26s-pose` | 768x1280 | 14.433 ms | 14.433 ms | 14.462 ms | 69.3 |
| `yolo26s-pose` | 384x640 | 4.782 ms | 4.784 ms | 4.818 ms | 209.0 |
| `yolo26s` detector | 384x640 | 4.605 ms | 4.605 ms | 4.615 ms | 217.1 |

Spread is tight: p99 sits within 4% of the median everywhere, and within 0.8% for every shape
except 768x1280.

## Results - full inference path

`benchmark_engines.py`, the same four levels the size sweep reported: `pre` is the Python
letterbox and blob build, `gpu` is copy plus enqueue plus sync, `post` is the residual
(`total - pre - gpu`) covering decode and NMS, `total` is one `infer()` call. Medians below, from
pass 2 of three; passes agreed on `gpu` within 0.5%.

| candidate | pre | gpu | post | total median | total p90 | dets |
|---|---:|---:|---:|---:|---:|---:|
| `x-pose` 384x640 | 3.355 ms | 20.711 ms | 1.741 ms | 25.683 ms | 26.535 ms | 2 |
| `x-pose` 416x640 | 3.850 ms | 23.758 ms | 2.923 ms | 30.536 ms | 31.227 ms | 2 |
| `x-pose` 640x640 | 4.980 ms | 34.069 ms | 3.610 ms | 42.514 ms | 42.705 ms | 2 |
| `x-pose` 768x1280 | 9.154 ms | 75.010 ms | 7.331 ms | 91.709 ms | 92.664 ms | 2 |
| `s-pose` 768x1280 | 3.861 ms | 17.808 ms | 4.030 ms | 25.700 ms | 25.770 ms | 1 |
| `s-pose` 384x640 | 1.223 ms | 5.610 ms | 1.358 ms | 8.195 ms | 8.227 ms | 1 |
| `s` detector 384x640 | 1.211 ms | 5.339 ms | 1.426 ms | 7.994 ms | 8.015 ms | 3 |

The two deployed models re-measured within 2% of the prior report (7.99 ms against 7.84 ms for the
detector, 8.20 ms against 7.80 ms for the keypoint model), so the two reports are on the same
footing.

Read the `pre` and `post` columns with care. See the caveat below: identical preprocess work
measured anywhere from 1.21 ms to 4.24 ms depending on process state, while `gpu` stayed stable
and agreed with `trtexec` within 3%.

## What the input shape buys

| shape | tensor pixels | raw GPU | vs 384x640 |
|---|---:|---:|---:|
| 384x640 | 245,760 | 19.944 ms | 1.00x |
| 416x640 | 266,240 | 23.593 ms | 1.18x |
| 640x640 | 409,600 | 32.916 ms | 1.65x |
| 768x1280 | 983,040 | 74.238 ms | 3.72x |

Cost tracks tensor pixels sublinearly: 4.0x the pixels costs 3.7x the time. There is no cliff to
exploit, so the only way `x` reaches a 1280-wide tensor inside a 33.3 ms period is by feeding it
less than a full frame.

That matters because `rgb_camera_migration_2026-09-09.md` calls `imgsz 1280` a floor, not a
preference: at the 1.2 m mount a 20 cm robot spans 110 px near the mat edge and 52 px at the far
corner, and a 640-wide tensor shows the network 37 px and 17 px instead. The plan left two ways
out, and step 0 was supposed to choose between them.

The numbers choose the smaller model. `yolo26s-pose` at 768x1280 costs 14.43 ms of GPU time, which
is 5.5 ms cheaper than `yolo26x-pose` at 384x640 and 60 ms cheaper than `x` at the same 768x1280.
A full-frame 1280-wide tensor is affordable today as long as the model is `s`. The crop path in
step 5f, batching native-resolution windows around each track, is the largest item in the plan and
latency no longer forces it.

## What the pose head costs

| model | shape | params | raw GPU |
|---|---|---:|---:|
| `yolo26x` detector, 2 classes | 640x640 | 58.81 M | 31.837 ms |
| `yolo26x-pose`, 4 classes, 2 keypoints | 640x640 | 62.64 M | 32.916 ms |

The pose arm costs 1.08 ms more, 3.4%, for a keypoint head, two extra classes, and 3.8 M extra
parameters. The plan's risk entry called the 21 ms estimate "an estimate of an estimate" because it
scaled a detector number by geometry and never measured a pose head. The pose head was not the part
worth worrying about.

## Contention

Solo timing is not what the application pays when two engines share the GPU. I ran pairs of
`trtexec` processes concurrently, each saturating its own queue for the same window.

| pair | engine | solo | concurrent | factor |
|---|---|---:|---:|---:|
| today's rig | `yolo26s` detector 384x640 | 4.605 ms | 9.616 ms | 2.09x |
| today's rig | `yolo26s-pose` 384x640 | 4.782 ms | 9.842 ms | 2.06x |
| `x` plus a second model | `yolo26x-pose` 384x640 | 19.944 ms | 45.487 ms | 2.28x |
| `x` plus a second model | `yolo26s-pose` 384x640 | 4.782 ms | 9.898 ms | 2.07x |

Two saturating queues roughly double each other, which is what serialization on one GPU looks like:
each engine waits out the other's work. The prior report fitted a single 1.5x in-batch factor to
JetPack 6 live runs and flagged that it conflated contention with a JetPack version speedup. The
2.06x measured here on one platform is the cleaner number for a saturating overlap.

The application does not saturate. It runs each model once per tick, so the realistic per-tick cost
is closer to additive than to 2x. For pose-only that means roughly 19.94 ms for the pose model plus
whatever the field mask model costs, and that second term is still unmeasured on this box.

## Gate B verdict

The gate: `yolo26x-pose` at the deployed input shape runs under 33.3 ms per inference on the Orin
NX, measured in the C++ application with no second model loaded.

**Passes on the engine-level measurement, at 384x640 and at 416x640. Fails at 640x640 and at
768x1280.** The application half of the gate cannot be run: the e-CAM25 module was destroyed on
2026-09-19 by a reversed FPC, so the box has no camera, and it builds with `BUILD_WITH_ZED=OFF` so
there is no SVO playback fallback. That is the same gap the prior report closed with, and it stays
open until the module is replaced.

What the engine-level numbers support:

- At 384x640 the pose model leaves 13.4 ms of raw GPU time inside a 33.3 ms period for the field
  mask model, the filter, navigation, and transmit. Counting host transfers, 12.6 ms.
- At 416x640, for a 1920x1200 16:10 sensor mode, that margin drops to 9.7 ms and the gate still
  holds.
- Heading refresh lands at 30 Hz or better solo, which is what decisions 2 and 3 in the plan
  assumed.
- 60 fps per-frame inference is out of reach at any shape: the 16.7 ms tick is under the 19.94 ms
  floor. The plan already assumed this and puts a motion detector on the frames the pose model
  misses.

## Caveats

- **No end-to-end measurement.** No `runner.tick`, no `pipeline.latency`, no loop rate. Engine-level
  timing plus a contention measurement is not a live run.
- **The field mask model is not on this box.** There is no `aarch64_sm87` DeepLab engine on the
  Jetson, so the one co-resident model the pose-only pipeline actually keeps went unmeasured. Build
  it and re-run the contention pair before treating the 12.6 ms of headroom as real.
- **The Python CPU stages are unreliable in this harness.** Identical preprocess work at 384x640
  measured 1.21 ms, 2.89 ms, 3.38 ms, and 4.24 ms in different processes, reproducibly within a
  process and stable across the three passes. Whichever candidate a process times first pays the
  inflated figure, and loading an engine changes the figure for the rest of that process. I did not
  chase the mechanism. The `gpu` column is unaffected and agrees with `trtexec` within 3%, so every
  claim above rests on GPU time, not on `total`.
- **The C++ path preprocesses differently anyway.** Per the prior report's stage mapping, the C++
  `preprocess_image` avoids the numpy transpose copy and its `decode_detections` should beat the
  Python NMS by the widest margin of any stage, so these `total` figures are an over-estimate of
  what the application pays.
- **Single frame, FP16 only.** One 1280x720 frame with 2 labelled robots. Postprocess cost scales
  with surviving box count, so a busier frame pushes `post` up for every row. INT8 stays ruled out
  on accuracy by `int8_quantization_2026-09-06.md`.
- **768x1280 was fed a 720p frame.** A real 1920x1200 source would letterbox from a larger image,
  so `pre` would grow. GPU time would not.
- **Build times are not comparable across shapes.** 7m32s, 10m09s, 2m19s, and 13m47s for 384x640,
  416x640, 640x640, and 768x1280. The 640x640 build was fast because the timing cache was warm by
  then, holding 33,944 entries.

## What this decides

- **The plan proceeds.** Step 0 was the cheap way to kill it and did not.
- **Train the `x` arm at 384x640, and build the rectangular engine beside the square one**, per
  step 2. Add a 416x640 export if the deployment camera lands on a 16:10 mode.
- **Add an `s-pose` arm at 768x1280 to the step 2 grid.** The plan already trains
  `s_d40000_cagehigh` as a size control at `imgsz 640`. A 1280 variant is the only arm measured
  here that satisfies both the `imgsz 1280` floor and the frame period, and it costs less than the
  `x` candidate does at 384x640.
- **Drop step 5f from the critical path.** Native-resolution crops were the fallback for getting
  `x` a 1280-wide tensor. An `s-pose` arm at 768x1280 gets there for 14.43 ms with no new batching,
  no full-frame cadence, and no cross-window NMS.
- **Measure the field mask engine next.** Build the `aarch64_sm87` DeepLab engine on the Jetson and
  re-run the concurrent pair against the pose candidate. That is the last engine-level number
  standing between this report and a defensible per-tick budget.
- **Re-run against a live tick when the camera is replaced.** Unchanged from the prior report.

## How the numbers were produced

```bash
# megamind: rectangular ONNX from the existing x arm, one per shape.
venv/bin/python training/yolo/convert_to_onnx.py \
  data/models/yolo26x-pose_d40000_2026-09-16_last.pt --imgsz 384 640 \
  -o data/models/yolo26x-pose_d40000_2026-09-16_last_rect384x640.onnx

# dev box: relay, because the Jetson cannot resolve megamind.
rsync -a megamind:'~/auto-battlebot/data/models/yolo26x-pose_d40000_2026-09-16_last_rect*.onnx' .
rsync -a ./ ben@192.168.50.147:~/auto-battlebot/data/models/

# Jetson: build, pin clocks, confirm the box is idle, then time.
venv/bin/python training/yolo/convert_to_tensorrt.py \
  data/models/yolo26x-pose_d40000_2026-09-16_last_rect384x640.onnx --workspace 2
sudo jetson_clocks
/usr/src/tensorrt/bin/trtexec --loadEngine=<engine> --noDataTransfers \
  --iterations=300 --warmUp=1000 --avgRuns=100
venv/bin/python training/model_eval/benchmark_engines.py \
  --candidate x384x640=<engine> --num-classes 4 --num-keypoints 2 \
  --frame data/bench_frame.png --iterations 300 --csv out.csv

# Contention: two saturating queues for the same window.
/usr/src/tensorrt/bin/trtexec --loadEngine=<a> --noDataTransfers --duration=20 --warmUp=2000 &
/usr/src/tensorrt/bin/trtexec --loadEngine=<b> --noDataTransfers --duration=20 --warmUp=2000 &
wait
```

`benchmark_engines.py` needed two new flags for this run. Its head-layout inference reads a
14-column output as 1 class and 3 keypoints, which is the same ambiguity `score.py` resolves from
`--labels`: 4 bbox columns plus 4 classes plus 2 keypoints is indistinguishable from 4 plus 1 plus
3 by width alone. `--num-classes 4 --num-keypoints 2` sets it, and the run now prints each engine's
resolved layout so the parse is visible in the log.

## Artifacts

- Per-stage CSVs: `assets/2026-09-19_pose_step0/xpose_shapes.csv` plus `_pass2`, `_pass3`,
  `spose.csv`, `spose_pass2.csv`, `sdet.csv`
- Console output: `assets/2026-09-19_pose_step0/xpose_shapes.txt`, `spose.txt`, `sdet.txt`
- Raw `trtexec` output: `assets/2026-09-19_pose_step0/trtexec_raw_gpu.txt`
- Contention logs: `assets/2026-09-19_pose_step0/contention/`
- ONNX (megamind and Jetson, not in git):
  `data/models/yolo26x-pose_d40000_2026-09-16_last_rect{384x640,416x640,640x640,768x1280}.onnx`
- Engines (Jetson only, not in git): the four `_aarch64_sm87.engine` files beside them, plus
  `yolo26s-pose_our_robot_keypoints_rect768x1280_2026-09-07_aarch64_sm87.engine`
- Benchmark tool: `training/model_eval/benchmark_engines.py`
