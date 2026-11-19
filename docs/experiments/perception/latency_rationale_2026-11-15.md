# Latency rationale for the control and publish paths

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/keypoint_model/yolo_keypoint_model.cpp`

Beside `{`:

Preprocess and inference share the engine's stream and synchronize once, at the end of inference. "preprocess" therefore times the frame upload (none for a pinned frame) and the kernel launch; the kernel's own GPU time lands in "inference".

### `src/perception_batch/parallel_model_batch.cpp`

Beside `constexpr std::chrono::milliseconds kUpdateWaitTimeout(1000);`:

Bound on how long update() waits for both models. ~30x the worst per-model time seen in latency reports; hitting it means a model is wedged, not slow.

### `src/publisher/async_publisher.cpp`

Beside `call = nullptr;`:

Destroyed before relocking: the call holds frames, and the last reference to a pinned frame frees it, which can be slow. Under the lock, enqueue() on the tick waited for it.

### `src/rgbd_camera/video_encoder.cpp`

Beside `constexpr const char *kFfmpegSoftware = "libx264";`:

Software. On the Orin it cannot hold a recording's 60 fps and takes the cores perception needs, so there it is only for the preview (VideoEncoderOptions::software_preview).

### `src/runner.cpp`

Beside `{`:

Gate perception against the field geometry before anything downstream sees it. The static gate runs first, and on robot blobs only. It has to see every detection to learn which field positions never move, so putting the height gate ahead of it would starve it of the floor-graphic detections it exists to catch. Our own robot comes from a different model and the filter tracks it regardless, so suppressing it for holding still would be wrong. The height gate then measures how far above the field plane each detection stands and records it on every keypoint, so projection stops assuming every robot is exactly keypoint_height_meters tall. It rejects on that measurement only when reject_enable is set: the height distributions of real robots and floor graphics overlap enough that rejecting costs real detections. See KeypointHeightGateConfiguration.

### `src/runner.cpp`

Beside `{`:

All publishing runs after the command send so none of it (notably the ~10 ms image compression on Jetson) sits on the control critical path. The live publisher is an AsyncPublisher, so this block only queues copies of the tick's outputs.

### `src/viz/image_encoder_worker.cpp`

Beside `job = Job{};`:

Freed before relocking: the last reference to a pinned frame frees it, which can be slow, and submit() on the tick takes this lock.
