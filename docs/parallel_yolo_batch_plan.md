# Plan: run keypoint + bounding-box YOLO models in parallel

## Goal

Run the two perception models — the keypoint model and the robot bounding-box/blob model
— **in parallel threads** instead of sequentially, sending data via queues and joining on
both completing each tick. The parallelization must obey the existing interface
architecture so either model stays swappable for any TensorRT implementation.

## Current behavior (sequential)

Both models already exist as separate swappable interfaces with identical shapes:

- `KeypointModelInterface` (`include/keypoint_model/keypoint_model_interface.hpp`):
  `initialize()`, `KeypointsStamped update(RgbImage)`, `DetectionsStamped last_detections()`.
  TRT impl `YoloKeypointModel`; noop `NoopKeypointModel`.
- `RobotBlobModelInterface` (`include/robot_blob_model/robot_blob_model_interface.hpp`):
  **structurally identical** — same three methods, same `RgbImage → KeypointsStamped`
  signature. TRT impls `YoloBboxRobotBlobModel` (active in `_desktop.toml`) and
  `YoloSegRobotBlobModel`; noop `NoopRobotBlobModel`.

Both wrap a `TrtEngine` by value (`include/tensorrt_inference/trt_engine.hpp`;
non-copyable/non-movable). They run **sequentially** in `Runner::tick()`
(`src/runner.cpp:392-402`), both fed the same `camera_data.rgb`:

```cpp
keypoints = keypoint_model_->update(camera_data.rgb);              // ~395
robot_blob_keypoints = robot_mask_model_->update(camera_data.rgb); // ~401
```

Outputs both feed `robot_filter_->update(...)` (:407) and are published via
`last_detections()` (:436-437).

**Constraints (CLAUDE.md):** no blocking calls in the main perception loop; <60 ms
end-to-end; the project uses `miniroscpp`, config-driven factory pattern.

## Threading reality (important)

Each model owns its own `TrtEngine` (independent CUDA context + device buffers), so two
engines on two threads are independent objects. **But** `TrtEngine::execute*`
(`src/tensorrt_inference/trt_engine.cpp:293,340`) uses blocking `cudaMemcpy` on the
default stream and `enqueueV3(nullptr)` — all on the default CUDA stream. So today,
parallel threads overlap the **CPU-side pre/post-processing** (letterboxing, NMS,
decoding) but serialize on the GPU default stream. This is still a latency win (pre/post
is non-trivial), and true GPU overlap is a follow-up (per-engine `cudaStream_t`). The
plan should state this explicitly so the win is measured, not assumed.

No thread-pool or concurrent-queue primitive exists yet. The codebase threading template
is the ZED camera: `std::thread` + `std::mutex` + `std::condition_variable` +
`std::atomic<bool>` done-flags + a `join_with_timeout(...)` helper
(`src/rgbd_camera/zed_rgbd_camera.cpp:63,292,322`). Follow that style.

## Approach

Introduce a small **parallel batch runner** that takes the two existing interfaces (by
`shared_ptr`), runs each `update()` on its own worker thread, and joins on both before
returning. Because it consumes the interfaces (not concrete classes), any TRT impl
remains swappable — the batch runner never names `YoloKeypointModel` etc.

### 1. Batch runner component

New `include/perception_batch/parallel_model_batch.hpp` +
`src/perception_batch/parallel_model_batch.cpp` (sources auto-globbed; no CMake edit):

- Holds two persistent worker threads (one per model) created at construction, each
  looping on a condition variable — **not** spawned per tick, to avoid per-frame thread
  creation cost in the hot loop. Mirror the ZED producer/consumer handshake.
- API:
  ```cpp
  struct BatchResult { KeypointsStamped keypoints; KeypointsStamped robot_blob_keypoints; };
  class ParallelModelBatch {
   public:
    ParallelModelBatch(std::shared_ptr<KeypointModelInterface> keypoint_model,
                       std::shared_ptr<RobotBlobModelInterface> blob_model);
    bool initialize();                 // calls initialize() on both
    BatchResult update(const RgbImage &image);  // dispatches both, joins, returns
    ~ParallelModelBatch();             // signals stop, join_with_timeout on both
  };
  ```
- `update()`: publish the shared `RgbImage` to both workers via the queue/condition-var
  handshake, each worker calls its `model->update(image)` and stores its result + sets a
  done atomic, then `update()` waits until both done atomics are set (the **join point**)
  and returns the combined `BatchResult`. Use `join_with_timeout`-style bounded waits so a
  hung model can't stall the loop forever.
- Threads share the input by const reference (both models only read `camera_data.rgb`);
  no per-thread image copy needed. Confirm both impls treat the input as read-only.

The "queues" requirement: a single-slot handoff per worker (input image in, result out)
guarded by mutex/condition-variable is the right-sized queue here — a full MPMC queue is
unnecessary for a fixed 2-producer/1-consumer-per-tick pattern. If a genuine
`std::queue`-based blocking queue is preferred, implement a minimal
`BlockingQueue<T>` in `perception_batch/` and use one per worker; note the tradeoff (more
general, marginally more overhead) in the PR.

### 2. Wire into the Runner

- `Runner` currently holds `keypoint_model_` and `robot_mask_model_` as `shared_ptr`
  members (`include/runner.hpp:66-69`), injected in `main.cpp:121-134`. Option A (minimal
  surface): construct a `ParallelModelBatch` inside `Runner` from the two injected models,
  and replace `src/runner.cpp:392-402` with a single `auto batch = batch_.update(camera_data.rgb);`
  then use `batch.keypoints` / `batch.robot_blob_keypoints` downstream (:407, :436-437).
- Keep the existing per-model `FunctionTimer` labels (`keypoint_model.update`,
  `robot_mask_model.update`) recorded **inside each worker** so the latency report still
  attributes per-model time, and add a `perception_batch.update` timer around the join to
  measure wall-clock of the parallel section.
- Preserve `last_detections()` publishing — the batch runner exposes the two models so the
  Runner can still call `keypoint_model_->last_detections()` / `robot_mask_model_->last_detections()`
  after `update()` returns (results are stored on the models as they are today).

### 3. Config (optional toggle)

Add a `[perception] parallel_models = true|false` (or a `RunnerConfig` field) so the
sequential path stays available as a fallback and for A/B latency comparison. When false,
`Runner` calls the two models sequentially as today. This de-risks the change and lets the
latency report quantify the win.

### 4. Keep it swappable — the interface contract

- The batch runner depends only on `KeypointModelInterface` / `RobotBlobModelInterface`,
  so swapping in `Noop*`, `YoloSeg*`, `YoloBbox*`, or a future TRT impl via TOML
  (`[keypoint_model] type=...`, `[robot_mask_model] type=...`) requires no batch-runner
  change. The factories (`make_keypoint_model`, `make_robot_blob_model`) are untouched.
- The two interfaces are identical in shape; consider (optionally) extracting a shared
  `ImageToKeypointsModel` base interface so the batch runner is generic over N models.
  Keep this optional — it touches two interface headers and their impls; only do it if it
  clearly simplifies. The `BatchResult` two-field approach avoids needing it.

## Testing

- Unit test `ParallelModelBatch` with two fake `*Interface` impls that sleep and return
  known results; assert both results returned, order-independent, and the timeout path
  triggers on a hung fake.
- Playback regression: `./scripts/build_and_run.sh -c config/playback.toml` — confirm
  identical detections vs. sequential mode (parallelization must not change outputs) and a
  reduced `perception_batch.update` time vs. summed sequential timers.
- Use `scripts/mcap_latency_report.py` (see the latency-report plan) to quantify the
  before/after on a recording.

## Validation

```bash
./scripts/build_and_test.sh --gtest_filter=ParallelModelBatch.*
git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy -p build-test/
./scripts/check_and_fix
```

## Open questions / follow-ups

- **GPU overlap**: today the default CUDA stream serializes GPU work. A follow-up should
  give each `TrtEngine` its own `cudaStream_t` and use async memcpy for real GPU-parallel
  execution. Flag as a separate task; the threading structure here is the prerequisite.
- **Thread lifetime**: persistent workers (recommended) vs. per-tick threads. Persistent
  avoids per-frame creation cost but needs clean shutdown (`join_with_timeout` in the
  dtor). Confirm this is acceptable.
- **Where the batch lives**: inside `Runner` (Option A) vs. a new injected component built
  in `main.cpp`. Option A is the smaller diff; a fully injected component is cleaner for
  testing. Recommend Option A first.
