# Depth per tick cost

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/rgbd_camera/zed_rgbd_camera.cpp`

Beside `if (!device_.retrieve(latest_data_)) {`:

Depth is retrieved for every frame. The ZED already computes it on every grab (RuntimeParameters::enable_depth is always on), so this only adds the measure retrieve and its copy, and it happens here on the capture thread rather than on the pipeline's.

### `src/rgbd_camera/zed_rgbd_camera.cpp`

Beside `if (!(frame_counter_ > last_returned_frame_counter_)) {`:

Every published frame carries depth, so there is one path: take the prefetched frame immediately, otherwise wait for the next one. Requesting depth and waiting for a frame that honoured the request used to cost up to two frame periods, because the capture thread reads the request before grab() and so an in-flight grab could never satisfy it. That halved the pipeline's frame rate while leaving per-tick latency looking healthy.

### `src/rgbd_camera/zed_svo_playback_camera.cpp`

Beside `const ZedDevice::GrabStatus status = device_.grab();`:

Grab on the caller's thread. Every frame reaches the pipeline, in order, and depth is retrieved for this frame rather than requested for a later one.
