# Workaround ledger

Every workaround the phase 1 comment audit found, with its root-cause fix where one is known. Fix
rows one at a time: the root cause goes, the workaround goes with it, and the row is deleted.

As of 2026-11-20 no comment defends any of these in code. Each reason now sits in a "Workarounds"
section of the subsystem's reference or setup doc. Where a batch extracted the workaround, the
function or constant name says what it guards. File line numbers below are from the 2026-11-15
audit and have moved since; search for the symbol. Built from the 322 `HACK` rows of the audit CSV,
deleted 2026-11-20 and still in git history at `docs/plans/comment_purge_audit.csv`.

## How to read it

- **Verified?** starts with `yes:`, `no:`, or `unverified:`. Auditors read code paths and installed
  library sources. They did not build C++, run the app, flash firmware, or touch hardware, so most
  `unverified` rows need one of those to settle.
- 124 rows are verified, 10 failed verification, 186 are unverified, and 2 carry no tag. 83 rows
  have no known root-cause fix.
- By area: `training/` 76, `firmware/` 39, `src/` 31, `tests/` 30, `hardware/` 27, `scripts/` +
  `install/` + `service/` + `web/` + `CMakeLists.txt` + `mypy.ini` + `docker/` 59, `playground/` +
  `simulation/` + `logo/` 31, `auto_battlebot/` 16, `include/` 13.

## Claims that failed verification

Phase 5 pilot, 2026-11-19. Each row got one confirming run.

| Location | Outcome |
| --- | --- |
| `training/deeplab/semantic_train.py:51` | Fixed: `seed_worker` deleted. torch 2.10 seeds Python `random` per worker; a 2-worker DataLoader drew 8 distinct values |
| `tests/camera/test_zed_svo_playback_camera.cpp:19` | Fixed: each test opens its own camera. Two tests already opened a second camera on the SVO with the shared one open; all 9 pass, 10 s to 16 s |
| `training/synthetic/pose_camera_server.py:47` (and `render_scenes.py`, `render_sprites.py`, `render_cage_view.py`) | Comments deleted, `sys.path` writes kept. `docker/run_synthetic.sh` no longer exists; CLAUDE.md records the Blender exception |
| `web/src/lib/image.ts:65` | Fixed: `CompressedMessage.data` is typed `Uint8Array<ArrayBuffer>`, so the Blob takes it without the `slice()` copy; `npm run check` passes |
| `mypy.ini:63` | Fixed: `[mypy-nhrl_common]` and `[mypy-nhrl_audit]` deleted; mypy passes (`mypy_path` itself was removed later) |
| `install/install_python_environment.sh:229`, `mypy.ini:17`, `mypy.ini:108`, `mypy.ini:112`, `tests/python/test_gpu_queue.py:18` | Fixed: the shared modules moved into `auto_battlebot/`; the `.pth` writer, `mypy_path` and both `ignore_missing_imports` sections are gone, and the test imports `auto_battlebot.gpu_queue` normally |
| `firmware/espnow_dongle/scripts/upload:2` | Fixed: defaults to `/dev/espnow_dongle`. Before, with both plugged in, it flashed the robot |
| `firmware/espnow_dongle/platformio.ini:25`, `firmware/mr_stabs_mk2/platformio.ini:30` | Claim wrong, overrides kept: the board file sets `use_1200bps_touch` and `wait_for_upload_port` true and `build_flags` never reach upload settings. Reasons in `docs/reference/robot_link/firmware_build.md` |

## Repeated workarounds

One root-cause fix retires several rows each.

- **ESP32 platform pin.** `espressif32@6.13.0` in `firmware/mr_stabs_mk2/platformio.ini:15`,
  `firmware/espnow_dongle/platformio.ini:8`, and the 250-byte frame limit in
  `espnow_protocol.h:22`, plus `DShot.cpp:8` writing into NeoPixel's private
  `rmt_reserved_channels[]`. Fix: port DShot to the IDF 5 `rmt_tx` driver.
- **libnvjpeg symbol clash.** Fixed: the dlopen plugin is the fix, and the comments went once a
  test guarded it. CTest `NoNvjpegLink` runs `scripts/check_no_nvjpeg_link.sh` on the direct
  `NEEDED` entries of `auto_battlebot_lib` and `auto_battlebot_test`, and a POST_BUILD step runs it
  on `auto_battlebot` and `viz_relay`. Any `libnvjpeg` or `libauto_battlebot_nvjpg` entry fails.
  Reasons are in `docs/reference/recording/foxglove_recording_format.md` ("NVJPG plugin").
- **Eigen `FromTwoVectors` `-Wmaybe-uninitialized` on GCC 13.** Fixed: the four sites call
  `rotation_between()` in `transform_utils`. It builds the quaternion directly with a
  fixed antiparallel axis, so no SVD runs and no `#pragma` is needed. Unit tests cover it.
- **ZED SDK 5.2 quirks.** `src/rgbd_camera/zed_one_rgb_camera.cpp` has five, including the
  setting nudge at `:394` and `_Exit` on a wedged `grab()` at `:51`. Most may go with an SDK upgrade.
- **BlenderProc segmentation pass indices.** Fixed: `synthgen/pass_index.py` `assign_pass_index()`
  runs as each mesh is created, and the distractor re-arm is gone. The 9 "must be first import"
  comments are gone too: BlenderProc enforces it (`SetupUtility.py:389`) and `pyproject.toml`
  keeps the order. Reasons in `docs/reference/tools/synthgen_render.md`.
- **ESP DTR/RTS reset.** Fixed: `auto_battlebot/robot_link/esp32_serial.py` `open_esp32_serial()`
  serves all three scripts (DTR and RTS high before `open()`); the reason is in
  `docs/reference/robot_link/espnow_link.md`. The `usb_dashboard.py` variant was not deliberate.
- **Camera retry and cancel.** Fixed: no camera's `initialize()` clears `cancel_open_` any more
  (it is set at construction only, and `cancel_initialize()` is terminal), and `Runner` no longer
  drains commands before the retry open.
- **`sys.path` routes around the package rule.** Fixed 2026-11-19. `gpu_queue` moved to `auto_battlebot/gpu_queue.py`, `seg_to_bbox` and `clear_image_cache` to `auto_battlebot/datasets/`, `gen_splash_animation` to `auto_battlebot/branding/splash_animation.py`, and the kinematic sim (`camera_utils`, `hazards`, `kinematic_sim_server`, `mujoco_window`, `policy_opponent`, `protocol`, `rl_eval_server`, `viewer`, `config/`, `plants/`) to `auto_battlebot/kinematic_sim/`. The old paths under `training/`, `logo/` and `simulation/` are thin entry points. The `auto_battlebot.pth` writer, the `mypy_path` line, the `seg_to_bbox` and `clear_image_cache` mypy sections, the test `sys.path` inserts and the path loaders are gone. Left alone: Blender-run scripts under `training/synthetic/` still import each other by top-level name, and `tests/python/conftest.py` `load_training` still loads `training/` CLI scripts by path.

## Bugs found during the audit

Defects in code, not workarounds.

Fixed 2026-11-19:

- `scripts/run_simulation.sh`: the fallback assigned the same missing per-robot config; it now
  falls back to `config/simulation/kinematic_sim.toml` (`86c44dab`).
- `firmware/velocity_jig/scripts/console.py` and `firmware/velocity_jig/README.md`: IMU scale now
  0.140 dps and 0.000488 g per LSB, matching `include/config.h`; a test pins the console to the
  header (`86c44dab`).
- `src/field_filter/point_cloud_field_filter.cpp`: the degenerate-plane fallback now returns
  (0, 0, 1), the same z >= 0 convention as the flip and `field_pose.cpp`, with tests (`75c48602`).
- `include/tensorrt_inference/trt_engine.hpp`: `ScopedQuietLogging` nesting is `thread_local`, so
  probing on one thread no longer demotes errors on others (`86c44dab`).
- `auto_battlebot/perception/trt_yolo.py`: `preprocess_frame` requires `letterbox_padding`
  (`86c44dab`).
- `hardware/cage_pack/camera_boot.py`: deleted (`86c44dab`).
- `CMakeLists.txt`: the ZED debug override passes `-U_DEBUG` instead of `-Wno-extra -Wno-error`;
  those sources build under `-Werror` (`86c44dab`).
- Missing docs: the test citation of `docs/robot_filter_decay_plan.md` was already gone; the stage0
  report now points at `our_robot_hold_window_s` in `docs/reference/config/profiles.md`. No
  remaining reference to `docs/velocity_jig_wiring.md` or `docs/hardware/README.md`.

Fixed 2026-11-19, found during phase 2:

- `src/transmitter/espnow_transmitter.cpp`: enabling autonomy while SA is already on now ramps
  from standstill; the first command after `enable()` used to skip the slew limit. Test
  `EspNowTransmitterPtyTest.ReEnablingWithSaOnRampsFromStandstill`.
- `include/robot_filter/ekf_math.hpp` `wrap_angle`: returns `(-pi, pi]` as documented; `-pi` maps
  to `pi`.
- `src/mask_model/deeplab_mask_model.cpp`: feeds RGB with the training corpus statistics. The
  audit found the ImageNet normalization; the eval found the bigger defect, 3-channel BGR frames
  passed to the network unswapped. Macro IoU 0.9219 to 0.9440
  (`docs/experiments/perception/2026-11-19_deeplab_preprocessing_fix.md`).
- `playground/opponent_prediction/run_replays.py`: start frames come from
  `config/playback/svo_start_frames.toml` through `auto_battlebot.recording.svo_start_frames`, so
  14-12-27 replays from 3000, not the script's own 1540. `make_eval_dataset.py` uses the same loader.
- `scripts/nhrl_to_recordings.py`: a clip skipped as already converted now advances the start
  stamp, so a re-run gives new clips the stamps a full run would. The one converted set on disk,
  `BrettZone_2026-05` (6 clips), had no overlapping stamps.
- `scripts/convert_ros1_mcap.py`: `--check` now flags a `/diagnostics` array whose log time
  reaches no per-module topic, so a dropped array fails the check; before, only a module with more
  messages than the source failed it.
- `src/mcap_recorder/mcap_recorder.cpp`: the recorder takes its output directory from
  `McapRecorderConfig::recordings_dir` (default `data/recordings`), and every recorder test writes
  to a per-test temp dir. Before, `HoldingDirectoryScanSurvivesAnUnreadableEntry` evicted real
  recordings (about 283 test MCAPs and 10 replay MCAPs on 2026-11-19) and the toggle tests left
  `toggle_*_test` files there. Test `McapRecorderToggleTest.EvictsOnlyFromItsOwnDirectoryAndNeverFromAnother`
  keeps a 1.1 GB sentinel outside the recorder's directory.
- `src/robot_filter/front_back_keypoint_converter.cpp` `get_pose_from_points`: the pose is a
  rotation about z by the front-back heading. It was the 3D shortest arc from +x, which for a
  robot facing near -x with the keypoints at different heights came out upside down (roll pi,
  yaw still right). Every consumer reads yaw only, so filtered poses do not change. Test
  `FrontBackKeypointConverterTest.PoseStaysUprightWhenTheKeypointsDifferInHeight`.

| Location | What the code does | What the comment claims would break | Verified? | Root-cause fix |
| --- | --- | --- | --- | --- |
| `CMakeLists.txt:153` | RTNeural populated without add_subdirectory, included as SYSTEM, no submodules | Its CMake would put bundled Eigen ahead of system Eigen and its headers fail -Werror | unverified: did not read RTNeural's CMakeLists | none known; an upstream option to use the system Eigen would remove it |
| `CMakeLists.txt:192` | re-runs find_package(ZED 5 REQUIRED) after QUIET | forced BUILD_WITH_ZED without SDK fails later with an unclear error | yes: the QUIET probe sets ZED_FOUND false and prints nothing | none known |
| `CMakeLists.txt:423` | CUDA flags pass GCC warnings through -Xcompiler and omit -O | nvcc rejects GCC flags and warns on duplicate -O, fatal under --Werror=all-warnings | unverified: comment claim | none known |
| `firmware/espnow_dongle/platformio.ini:8` | Pins platform = espressif32@6.13.0 (Arduino core on IDF 4.4). | An unpinned espressif32 resolves to the globally installed pioarduino 55.x (IDF 5), which changes ESP-NOW frame limits the wire format assumes. | unverified: matches project memory note pioarduino-shadows-espressif32; did not build unpinned. | Port to IDF 5 (ESP-NOW v2 frames, rmt_tx DShot on mr_stabs_mk2) and pin to that platform explicitly. |
| `firmware/mr_stabs_mk2/platformio.ini:15` | Pins platform = espressif32@6.13.0 (Arduino core 2.x, IDF 4.4). | pioarduino 55.x (IDF 5) removes NeoPixel's rmt_reserved_channels and deprecates the legacy RMT driver DShot.cpp uses. | yes: DShot.cpp:10,62 use rmt_reserved_channels from .pio/libdeps/local/Adafruit NeoPixel/esp.c:154 and the legacy rmt_config/rmt_write_items API; did not build on IDF 5. | Port DShot to IDF 5 rmt_tx/rmt_rx with its channel allocator. |
| `hardware/zed_box_mini_mount/zed_x_one_s_assembly.py:102` | Places the ZED Box Mini STEP at z = adapter thickness - 12.0. | out/step/zed_box_mini.step was exported on the old 12 mm adapter, so placed as-is it sits 5.3 mm above the current adapter. | yes: zed_mount.py docstring and cage_layout.ZED_STEP_BASE_Z = 12.0 both record the STEP base sheet at z 12. | Re-export zed_box_mini.step from assembly_page.py with its base sheet at z = 0 and drop the offset here, in zed_mount.py and in cage_layout.ZED_STEP_BASE_Z. |
| `include/tensorrt_inference/pinned_mat_allocator.hpp:29` | instance() returns a function-static pointer from new that is never deleted | cv::Mats that outlive static destruction would call deallocate on a destroyed allocator and its mutex/pool | unverified: src/tensorrt_inference/pinned_mat_allocator.cpp:8-11 leaks as described; frames are held by camera and publisher threads, but I did not find a static-lifetime Mat or reproduce the crash | Give frame Mats an owner with explicit teardown before exit (camera/publisher joined and buffers released), then a plain function-static allocator works; otherwise none known |
| `src/rgbd_camera/video_encoder.cpp:84` | When no FFmpeg encoder opened and software is allowed, opens GStreamer x264enc instead. | NVIDIA's JetPack 7 FFmpeg ships without libx264, so the software path would open nothing. | unverified: code path read; matches the dashboard-kiosk memory that the Orin Nano preview uses the x264enc fallback; did not inspect the Jetson FFmpeg build | Build or vendor an FFmpeg with libx264 on JetPack 7, or make GStreamer the only software path and drop the FFmpeg libx264 branch |
| `src/rgbd_camera/zed_device.cpp:33` | Moves a timed-out std::future from zed_.open() into a never-freed static vector. | Destroying a std::async future blocks on its task, which is wedged inside the ZED SDK, so shutdown would hang. | yes: std::async futures block in their destructor until the task finishes (standard behavior); the SDK hang itself is not reproduced. | none known; zed_.open() cannot be interrupted. Running open in a detached std::thread or a child process would avoid the leak bookkeeping. |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:217` | Retrieves rectified BGRA and converts to BGR with cvtColor each frame. | VIEW::LEFT_BGR does not exist on the ZED Box Mini's SDK 5.2. | unverified: code read; SDK version claim not checked against SDK headers | Upgrade the ZED Box to an SDK with LEFT_BGR, or keep BGRA end to end since the encoder already takes BGRA |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:28` | Compiles AE_ANTIBANDING handling only when the ZED SDK is 5.5 or newer. | On the ZED Box Mini's SDK 5.2 the control does not exist; the build would fail. | yes: AB_ZED_HAS_ANTIBANDING guards lines 358, 382, 429; ZED Box runs SDK 5.2 per docs/setup/zed_box_mini.md | Upgrade the ZED Box Mini to SDK 5.5 or newer and drop the guard |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:340` | read_settings substitutes the last applied exposure and gains when the SDK reports -1. | SDK 5.2 on the ZED X One reports -1 in auto and manual; showing or resending -1 clamps exposure to 28 us and gives a black frame. | unverified: get_or lambda confirmed in code; the SDK behaviour and 8.7 vs 50.6 green measurement were not reproduced | Upgrade the ZED Box SDK if a later release reports these settings |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:394` | Before each setCameraSettings, writes an adjacent value so the real value is always a change. | SDK 5.2 skips a write equal to the last one, so reapplying a profile after DEFAULT left auto exposure running (measured brightness 86 vs 8). | unverified: nudge confirmed in the set lambda; SDK dedupe behaviour not reproduced | Upgrade the SDK, or explicitly toggle AEC_AGC off before applying manual values if that defeats the dedupe |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:51` | Destructor waits 3 s for the capture thread, then calls std::_Exit if it is stuck in grab(). | Joining hangs until systemd kills the process 90 s later; detaching leaves the thread in grab() on a freed camera. | unverified: read destructor and Runner::stop_recordings_for_shutdown (runner.cpp:175, called at 329/651/691 before teardown) which supports the recordings-closed claim; grab() hang itself not reproduced | Interruptible grab (SDK grab timeout or a close from another thread that unblocks grab), so the thread can be joined |
| `src/runner.cpp:289` | wait_for_frames_in_flight sleeps until three more ticks have started (or 500 ms) before the MCAP gate closes on a recording-off. | A tick holding a frame captured before the encoder stopped would publish its frame_meta, with the old video index, into the next file and point at the wrong packet. | unverified: frame_meta is logged from data.frame_identity in src/publisher/foxglove_publisher.cpp:220 with no file generation check, so the claim is consistent with the code; not reproduced. | Stamp each frame_identity with the recording generation and have the publisher drop frame_meta whose generation does not match the open file, then delete the tick-count wait. |
| `src/tensorrt_inference/trt_engine.cpp:18` | Defines AB_TRT_HAS_STREAM_READER_V2 from NV_TENSORRT_MAJOR/MINOR and compiles either an IStreamReaderV2 or an IStreamReader implementation. | TensorRT before 10.7 lacks IStreamReaderV2, and 10.7+ deprecates IStreamReader. | yes: CLAUDE.md platforms list JetPack 6.2 with TensorRT 10.3 as supported; the comment is stale in saying the Jetson predates 10.7, since JetPack 7 ships 10.16. | Drop JetPack 6 support, or keep the shim and fix the comment; the guard itself still needs to exist while JP6 boxes (ZED Box Mini) are deployed. |
| `src/tensorrt_inference/trt_engine.cpp:27` | A process-wide atomic depth counter demotes all TensorRT logger errors to debug while any ScopedQuietLogging guard lives; EngineSelector holds one while deserializing candidates (src/engine_selector/engine_selector.cpp:49). | TensorRT logs a misleading version-mismatch error for each foreign-GPU candidate into stdout and the recording. | yes: read trt_engine.cpp:25-50,193-198 and engine_selector.cpp:49. Note the counter is global, so a real TensorRT error from another thread during probing is also demoted. | Read the compute capability from the plan header (or skip by sm tag) before calling deserializeCudaEngine, so foreign engines never reach TensorRT; or make the quiet flag thread_local. |
| `src/transmitter/opentx_transmitter.cpp:308` | When the serial port is open but no channel frames arrived for kChannelTimeout, closes the port so reconnect_if_needed reopens it and re-sends telemetry on / channels on. | Re-priming the OpenTX channels stream needs a fresh open; writing channels on to the still-open port does not restart it. | unverified: radio firmware behavior, not checkable from the repo. The code only ever writes the priming commands inside the open path (lines 64-65), so it never tried re-priming on an open port. | Try writing channels on to the open port first and close only if that fails to restart the stream; if the radio truly needs a reopen, none known. |
| `src/viz_relay/main.cpp:290` | Starts a detached thread that calls std::_Exit(0) if shutdown has not finished within kShutdownDeadline (2 s). | foxglove::WebSocketServer::stop() waits for every client to close; a client whose link dropped never answers, so the relay hung until systemd killed it after 90 s. | unverified: read the shutdown path; did not read the Foxglove SDK stop() implementation or reproduce the iPad link drop | A stop() with a close timeout in the Foxglove SDK, or set TCP keepalive/user timeout on client sockets so dead clients fail fast |
| `tests/camera/test_gst_h264_backend.cpp:66` | Feed loop retries encode() with a 2 ms sleep between frames, pacing frames like the camera. | Without the 2 ms pacing the backend drops queued frames and the every-frame-comes-back assertion fails. | unverified: did not run on a Jetson; read the comment against the encode() retry loop only | Make the backend's drop-when-queued guard configurable per session so the test can disable it instead of pacing. |
| `tests/camera/test_video_encoder.cpp:22` | Fake backend sleeps 2 ms in every encode() so frames pile up behind the refused one. | Without the sleep the queue drains before the refusal, so the refused-frame-stops-session test passes without exercising queued frames. | yes: read the fake and RefusedFrameStopsTheSessionInsteadOfShiftingIndices; without the delay the queue drains before refuse_pts_ | Gate the fake on a condition variable released by the test instead of a timed sleep. |
| `tests/camera/test_video_encoder.cpp:76` | Test branches on codec_name(): for nvv4l2h264enc it accepts the requested keyframe at index 7 or 8, for every other encoder exactly 8. | nvv4l2h264enc applies force-IDR to the next frame the hardware encodes; when the hardware is a frame behind, the IDR lands one frame before the requested one, so an exact-index assertion fails on the Jetson. | unverified: GstH264Backend::on_encoder_input (src/rgbd_camera/gst_h264_backend.cpp:262-265) emits force-IDR from the encoder sink pad probe when the requested PTS arrives, which fits the claim, but the one-frame lead is hardware behaviour I cannot run here. | Emit force-IDR one buffer later (on the probe for the frame after the requested PTS) or use the encoder's per-buffer GstVideoCodecFrame keyframe flag, then assert the exact index on all encoders. |
| `tests/camera/test_video_playback_camera.cpp:243` | Asserts frames >= kFrames - 2 instead of == kFrames. | An exact frame-count assertion fails when the decoder swallows the first one or two packets while priming. | unverified: decoder priming behaviour taken from the comment; did not run the test | Flush the decoder at end of stream (send EOS and drain) so every packet comes out, then assert the exact count. |
| `tests/camera/test_zed_svo_playback_camera.cpp:19` | Suite opens one static ZedSvoPlaybackCamera in SetUpTestSuite and shares it across tests instead of opening the SVO per test. | Opening the same SVO twice in one process segfaults the ZED SDK. | no: contradicted in the same file. SvoEndOfFile constructs and initializes a second ZedSvoPlaybackCamera on the same test.svo2 while shared_camera_ is still open (released only in TearDownTestSuite). Either the segfault no longer happens or it depends on open order; not run (no ZED build here). | Run the suite with a per-test camera; if it passes, drop the shared static. If it crashes, the fix is in ZedSvoPlaybackCamera teardown (close the sl::Camera before the next open), not a shared fixture. |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:104` | Calls recorder.flush() after set_enabled(false) before reading the closed file. | Without the flush read_file(first) runs before the writer thread finalizes the file and has_summary is false. | yes: the test reads the first file right after; set_enabled(false) queues the close to the writer thread | Have set_enabled(false) return a future or block until the file is finalized. |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:134` | Runs the toggles on a detached thread and waits on a future with a 5 s timeout. | Run inline, a reintroduced McapLogSink/recorder lock deadlock hangs ctest instead of failing this test. | yes: lines 142-151 detach the thread and abort on timeout | none known; a hang guard is inherent to a deadlock regression test (ctest TIMEOUT is the alternative). |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:150` | Calls std::abort() after ADD_FAILURE when the toggle thread does not finish. | Returning normally on timeout runs fixture teardown against a recorder the deadlocked thread holds, hanging or crashing the suite. | yes: the detached thread captures the recorder shared_ptr and is still blocked inside it | Run deadlock tests in a death-test subprocess (EXPECT_EXIT) so a hang kills only the child. |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:154` | Flushes the async spdlog logger before closing the recorder. | Without the flush recorder->close() can run before the async logger delivers 'after reopen', so count_by_topic['/log'] is 0 and the test flakes. | yes: setup_logging installs an async logger; the assertion counts /log messages in the closed file | Have McapRecorder::close() flush registered log sinks, or use a synchronous logger in tests. |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:194` | Sleeps 20 ms after stop->store(true) before restoring the default logger. | Without the sleep the background logging loop may still log into the recorder's sink after close()/logger restore, a use-after-close race. | unverified: the logging thread is detached, so nothing joins it; the sleep is a guess at its exit time | Keep the std::thread and join it instead of detaching and sleeping. |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:249` | Asserts the worst write latency is under 50 ms. | Tighten the bound toward 1 ms and the test flakes on a loaded box; drop the reason and someone loosens it past a real close wait. | unverified: timing bound; did not measure on a loaded box | none known; a timing assertion needs headroom. An instrumented hook that records whether write() waited on the close would remove the timing. |
| `tests/mcap_recorder/test_mcap_recorder_toggle.cpp:299` | Flushes before hold_writer so the file is open first. | Without the flush hold_writer can stop the writer before the file exists, so the later read_file(path) finds no file or no header. | yes: hold_writer freezes the writer thread; the test later reads the file at path | Open the file synchronously in the constructor or set_enabled(true). |
| `tests/perception_batch/test_parallel_model_batch.cpp:147` | Sleeps 500 ms so the 1300 ms straggler finishes before the next batch.update(). | Without the sleep the next update() races the 1300 ms straggler still in the blob model, so the recovered tick can also time out and the recovery assertion flakes. | yes: the slow model sleeps 1300 ms once; the batch wait deadline is shorter, so the worker is still busy | Expose a way to wait for idle workers (batch.wait_idle()) instead of a fixed sleep. |
| `tests/publisher/test_async_publisher.cpp:136` | Bounds flush time by 5 ms x kMaxPendingCalls. | A bound tighter than kMaxPendingCalls x 1 ms flakes; a looser one stops catching a flush that waits for an empty queue. | yes: inner->delay is 1 ms per call and the queue caps at kMaxPendingCalls | none known; a timing bound needs headroom. |
| `tests/rl_policy/test_rtneural_runtime.cpp:241` | Asserts median inference under 2000 us, far above the expected value. | A ceiling near the real median fails on a loaded dev box (load avg 25+ inverts timings). | yes: the test prints the median and the bound is loose by design | Move timing to a benchmark target that reports but never fails. |
| `tests/tensorrt_inference/test_gpu_letterbox.cpp:175` | Measures pool counts before and after the release instead of absolute counts. | Asserting absolute pool counts fails depending on which tests ran first in the process. | yes: the pinned pool is a process-wide singleton shared by every test in the binary | Give the pool a reset or per-test instance so the test can assert absolute counts. |
| `tests/tensorrt_inference/test_gpu_letterbox.cpp:76` | expect_close allows one grey level per sample and a tighter mean bound. | An exact-match comparison against the CPU letterbox fails by up to one grey level per sample. | unverified: cv::resize 11-bit weight quantization taken from the comment | Compare against a float reference resize instead of cv::resize on 8-bit images. |
| `tests/tensorrt_inference/test_gpu_letterbox.cpp:93` | random_image blurs the noise with a 5x5 Gaussian before the comparison. | Without the blur, interpolation differences on white noise exceed the mean tolerance in expect_close. | unverified: did not run the test without the blur | Compare against a float reference so interpolation on noise matches exactly. |
| `tests/tensorrt_inference/test_trt_engine_graph.cpp:15` | find_engine() returns empty and the test GTEST_SKIPs when no engine for this sm/TRT exists. | Failing on a missing engine breaks the suite on every box without that sm/TRT-specific file. | yes: lines 31 and 36 skip on no engine and on load failure | Build a tiny engine from an ONNX in the test so it runs on every GPU box. |
| `tests/test_config.cpp:1250` | Writes a [plant] table into a keypoint-height config test. | Drop [plant] and the config load throws on the motion estimator before keypoint heights are parsed, so the test fails for an unrelated reason. | yes: the comment says the default dead-reckoning estimator requires [plant]; config parse throws without it (see RobotFilterMotionEstimatorPlantMissingFieldThrows) | Let the test pick a motion estimator that needs no plant, or provide a test helper that adds the plant table. |
| `tests/test_control_loop.cpp:319` | Asserts the cycle rate is tracked instead of an exact 8-per-frame count. | An exact 8-cycles-per-frame assertion fails on accumulated floating-point rounding. | yes: 240 Hz over 1/30 s frames is 8 periods, and floor of accumulated time gives 7 or 8 | Use integer tick arithmetic (frame stamps in ns, rate as an integer period) so the count is exact. |
| `tests/test_ekf_math.cpp:16` | Compares \|wrap_angle(x)\| to pi instead of the signed value for odd multiples of pi. | EXPECT_NEAR(wrap_angle(3*pi), pi) fails when atan2 returns -pi for the same heading. | yes: wrap_angle uses atan2(sin, cos), whose sign at +/-pi follows the residual sine | Define wrap_angle to return exactly pi for the boundary (wrap into (-pi, pi]). |
| `tests/test_jig_plant_model.cpp:16` | Compares C++ plant output to the Python golden fixture within 1e-9. | An exact-equality comparison fails on libm last-ulp differences between C++ and numpy. | yes: both step 2 ms substeps over 1200 steps; last-ulp libm sin/cos differences accumulate | none known; cross-runtime float comparison needs a tolerance. |
| `tests/viz/test_viz_sink.cpp:167` | Polls sink.connected() up to 1 s before publishing. | Publishing immediately after accept races the sink's connect handshake and the payload never reaches the fake relay. | unverified: did not read VizSink's connect state machine | Make publish() before connect queue the latched message, or have connect() block until the handshake completes. |
| `training/synthetic/docker/entrypoint.sh:18` | Makes /root world-traversable and symlinks the baked Blender install into /home/bot before dropping privileges. | BlenderProc resolves Blender at $HOME/blender, so the dropped user would not find the baked install and would re-download Blender. | unverified: BlenderProc's default install path is $HOME/blender per its docs; not run here | Bake Blender at a shared path such as /opt/blender and launch with blenderproc run --custom-blender-path /opt/blender, then drop the chmod and symlink |
| `training/synthetic/docker/entrypoint.sh:24` | chowns the top-level HF_HOME directory to HOST_UID before the privilege drop. | The baked HF cache is root-owned, so the dropped user cannot write to it. | unverified: follows from the Dockerfile baking HF_HOME as root; not run | Bake HF_HOME world-writable or always mount a host cache over it |
| `training/synthetic/docker/warmup.py:1` | A no-op blenderproc run at image build so setup_pip writes a complete installed_packages_cache_v2.json. | blenderproc pip install records only the named package, so every runtime run re-checks the default packages with pip. | unverified: BlenderProc setup_pip internals not read | Upstream: have blenderproc pip install record default packages too; or pin the cache file in the Dockerfile |
| `training/synthetic/pose_camera_server.py:50` | Adds the script directory and repo root to sys.path at import time. | Blender's embedded Python ignores PYTHONPATH, so first-party modules would not import. | no: contradicted inside this area; docker/run_synthetic.sh:97-100 exports PYTHONPATH=/workspace/training/synthetic and says that replaces in-script sys.path writes, while CLAUDE.md and this comment say Blender ignores PYTHONPATH. One of the two is stale | Settle which is true for the pinned Blender build; if PYTHONPATH works, drop the sys.path write, otherwise drop the PYTHONPATH export and use one shared bootstrap helper |
| `training/synthetic/prepare_robot_model.py:16` | Records the launch working directory from BLENDERPROC_CWD or os.getcwd() at import. | BlenderProc changes CWD to a temp directory, so relative paths given on the command line would resolve inside it. | unverified: BlenderProc internals not read; the BLENDERPROC_CWD fallback suggests the env var is set by the launcher | Resolve all CLI paths to absolute in the launcher (run_synthetic.sh) before invoking blenderproc |
| `training/synthetic/prepare_robot_model.py:396` | Falls back to bpy.data.materials for CC materials BlenderProc did not return. | BlenderProc's loader returns only newly loaded assets, so a second robot in the same run gets no materials. | unverified: BlenderProc load_ccmaterials not read | Cache the loaded materials by name once per run in this script instead of re-calling the loader per robot |
| `training/synthetic/prepare_robot_model.py:471` | Wires PBR maps from the cc_textures directory directly into the slot material when the CC material looks flat. | Some Blender/asset combinations load CC entries with no effective textures, so robots would render flat. | unverified: the comment names no specific Blender version or asset; not reproduced | Identify the failing asset/version pair; if it is a missing-suffix asset, fix it in download_ambientcg KEEP_SUFFIXES, otherwise always use the direct wiring path and drop the CC loader |
| `training/synthetic/render_cage_view.py:40` | Writes the script directory (and repo root) into sys.path at import time. | Blender's embedded Python ignores PYTHONPATH. | unverified: conflicts with docker/run_synthetic.sh:97-100, which exports PYTHONPATH as the replacement for in-script sys.path writes. render_scenes.py says BlenderProc clears it, which would make the export dead | Confirm whether blenderproc run forwards PYTHONPATH; if it never does, delete the export from run_synthetic.sh and move the sys.path bootstrap into one shared module |
| `training/synthetic/render_scenes.py:37` | Writes the script directory (and repo root) into sys.path at import time. | BlenderProc clears PYTHONPATH before launching Blender. | unverified: conflicts with docker/run_synthetic.sh:97-100, which exports PYTHONPATH as the replacement for in-script sys.path writes. render_scenes.py says BlenderProc clears it, which would make the export dead | Confirm whether blenderproc run forwards PYTHONPATH; if it never does, delete the export from run_synthetic.sh and move the sys.path bootstrap into one shared module |
| `training/synthetic/render_sprites.py:50` | Writes the script directory (and repo root) into sys.path at import time. | Blender's embedded Python ignores the PYTHONPATH run_synthetic.sh exports. | unverified: conflicts with docker/run_synthetic.sh:97-100, which exports PYTHONPATH as the replacement for in-script sys.path writes. render_scenes.py says BlenderProc clears it, which would make the export dead | Confirm whether blenderproc run forwards PYTHONPATH; if it never does, delete the export from run_synthetic.sh and move the sys.path bootstrap into one shared module |
| `training/synthetic/render_sprites.py:59` | Captures BLENDERPROC_CWD or os.getcwd() at import as LAUNCH_CWD. | BlenderProc re-executes the script from a temp directory, so relative paths resolve wrongly. | unverified: same claim as prepare_robot_model.py:15; BlenderProc internals not read | Resolve CLI paths to absolute in the launcher before blenderproc run |
| `training/synthetic/synthgen/cage.py:520` | Applies roll_deg about the model x axis to the house bot GLB parent before measuring its footprint. | The GLB ships lying on its side, so it would render on its side and the footprint would be measured wrong. | unverified: asset not inspected | Re-export the house bot GLB upright and set roll_deg to 0 |
| `web/src/lib/connection.ts:110` | Silence watchdog: after SILENCE_MS with no status, probes /healthz and drops the socket if the app is connected | Safari hands back dead sockets after unlock or link loss with no close event for minutes; the dashboard freezes | unverified: relies on /status/system at 10 Hz, consistent with download_recordings.sh's 10 Hz assumption | Relay-level WebSocket ping/pong with a client timeout, if the Foxglove SDK exposes it |
| `web/src/lib/connection.ts:257` | casts WebSocket as unknown as IWebSocket | DOM send() type is narrower than IWebSocket's | unverified: tsc not run | upstream @foxglove/ws-protocol typing fix |
| `web/src/lib/connection.ts:38` | Offers foxglove.sdk.v1 alongside the client library subprotocol | The SDK server returns 400 without foxglove.sdk.v1 | unverified: did not check the SDK server handshake code in third_party/foxglove | Upgrade @foxglove/ws-protocol to a version that offers foxglove.sdk.v1 itself, or none known |
| `web/src/lib/connection.ts:42` | Abandons a WebSocket handshake that has not opened after a timeout | iOS takes over a minute to fail an unanswered SYN after cable unplug, and nothing retries | unverified: iOS behaviour not reproduced | none known (browser TCP behaviour) |
| `src/runner.cpp:68` | `quit_from_signal_handler` calls `camera_->cancel_initialize()` because it only stores an atomic | An override that locks a mutex or logs would deadlock the signal handler on Ctrl-C during a camera re-open | unverified: async-signal safety of an arbitrary override cannot be tested; `QuitCancelsACameraReopen` covers the behavior only | none known (interface contract) |
| `src/runner.cpp:587` | `static_gate_->reset()` after field init | Static-gate clusters live in field coordinates, so a re-init leaves them pointing at the wrong place and the gate suppresses real detections | unverified: no test double for `FieldFilterInterface::compute_field` that returns a plane, and `StaticDetectionGate` exposes no cluster count | add a fake field filter and a cluster count accessor |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:65` | Waits on `pending_open_` before `zed_.close()` | `sl::Camera::close()` during `open()` on another thread crashes at shutdown | unverified: needs a ZED camera | none known |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:148` | Encoder callback stamps `CompressedVideo` with `log_time_ns` | Foxglove's decoder rejects a frame whose timestamp is not after the previous one | unverified: needs the Foxglove decoder in the loop | none known |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:182` | `start_encoder()` failure throws when recording is on | A session recorded without video cannot be replayed | unverified: needs a ZED camera | none known |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:225` | Fresh pinned `cv::Mat` per frame | Reusing one Mat overwrites a frame the pipeline still holds by reference | unverified: needs a ZED camera | none known |
| `src/rgbd_camera/zed_one_rgb_camera.cpp:241` | `camera_info.header` copied from the image header | Foxglove image panels reject a calibration whose frame differs from the image's | unverified: needs a ZED camera | none known |
| `src/robot_filter/kalman_motion_estimator.cpp:516` | Our-robot forward speed corrects in a second `ekf_update` with its own gate | Sharing the gate lets one noisy finite-difference speed discard the position fix that arrived with it | unverified: the failing arm is a structural change (a joint 4-row update), so there is no one-line mutation, and a gated speed row needs a hand-tuned scenario | build the joint-update arm behind a test-only switch and compare |
| `src/robot_filter/kalman_motion_estimator.cpp:717` | Opponent diagnostics omit infinite heading and yaw-rate sigmas | A heading-blind track would write inf into the diagnostics stream | unverified: `DiagnosticsLogger` is a process-global with no capture backend in the test binary | a capturing `DiagnosticsBackend` fixture |
| `hardware/mr_stabs_mk2_pcb/pcba/design.py:372` | Adds two GND_STITCH via footprints as netlist parts in the stem sides. | The pcba-design flow stitching pass finds no via spot in the bottom GND pour fragments boxed in by the USB-C and bulk cap tracks. | unverified: needs a routing run of the pcba-design flow. | Teach the flow stitcher to place vias in small pour fragments, or reroute the USB-C and bulk-cap tracks so the pour is not split. |
| `hardware/mr_stabs_mk2_pcb/pcba/gen_placement.py:448` | Adds a B.Cu track and via keep-out over the VBATT pour neck between the shunt and ESC_L+. | Two routing tries cut the neck with signal tracks, so 20 A ESC current flowed through a track. | unverified: needs a routing run. | Add a post-route check that the VBATT copper between shunt and ESC+ pads has a minimum width (rating_check.py), so any cut fails the flow instead of relying on a keep-out. |
| `hardware/mr_stabs_mk2_pcb/pcba/gen_placement.py:458` | Adds a small B.Cu keep-out strip between R3 VBATT pad and ESC_L+. | SENSE_N and SENSE_P tracks boxed in R3 VBATT pad so the pour no longer joined the Kelvin tap. | unverified: needs a routing run. | Connectivity check that R3 pad 2 touches the VBATT pour, or route the Kelvin taps as fixed pre-routes. |

## TEST rows deferred

Rows from the `TEST` verdict batches that could not get a test in a headless build. The comment is
gone and the claim is in a reference doc; the test is still owed.

| Location | What the code does | What the comment claims would break | Verified? | Root-cause fix |
| --- | --- | --- | --- | --- |
| `src/tensorrt_inference/trt_engine.cpp:35` | probe loads of EngineSelector candidates log below error level | A rejected candidate writes error lines to stderr and the MCAP /log topic | unverified: needs a serialized engine for the installed TensorRT plus a log capture | A fixture engine built at test time and a capturing logger |
| `src/tensorrt_inference/trt_engine.cpp:63` | stream reader stages every read through a host buffer and cudaMemcpyDefault | A plain memcpy into a V2 device destination segfaults or corrupts deserialization | unverified: the reader sits in an anonymous namespace and needs a CUDA device and a TensorRT V2 caller | Expose the reader to tests and drive it with a device destination |
| `src/tensorrt_inference/trt_engine.cpp:91` | cudaMemcpyAsync from staging_ followed by a stream sync before return | The next read overwrites staging_ under an in-flight copy and corrupts engine bytes | unverified: same reader dependency | Same seam |
| `src/tensorrt_inference/trt_engine.cpp:109` | seekg is preceded by clear() | A read that hit EOF leaves failbit set and seekg is a no-op | unverified: same reader dependency | Same seam |
| `src/tensorrt_inference/trt_engine.cpp:150` | destructor assumes execute() synchronized the stream | An execute() that returns early leaves copies in flight when the stream is destroyed and pinned buffers freed | unverified: needs a loaded engine on a CUDA device | A fixture engine built at test time |
| `src/transmitter/opentx_transmitter.cpp:60` | initialize() writes `telemetry on` and `channels on` on every open | After an unplug and replug the receive side stays silent until enable() is called with the port open | unverified: the radio is found by USB VID/PID and serial_ is a concrete SerialPort with no injection point | A SerialPort interface with a loopback fake |
| `src/transmitter/opentx_transmitter.cpp:66` | last_channel_time_ is set at open | Every healthy connect flashes a trainer-mode fault until the stall timeout | unverified: same serial dependency | Same seam |
| `src/transmitter/opentx_transmitter.cpp:112` | enable()/disable() gate only trainer output | With autonomy disabled link stats stop updating and the radio init button stops working | unverified: same serial dependency | Same seam |
| `src/transmitter/opentx_transmitter.cpp:119` | disable() zeroes the slew-limiter state | Re-enabling jumps straight to the last commanded speed instead of ramping from zero | unverified: send() returns early without an open port | Same seam |
| `src/transmitter/opentx_transmitter.cpp:278` | log_switch_states stamps the section on every frame, not on change | A staleness check reads a live link as dead because the section age grows while frames arrive | unverified: same serial dependency, plus a process-global DiagnosticsLogger | Same seam and a capturing DiagnosticsBackend |

## Rows that could not be tested (phase 2, 2026-11-19)

The comment is gone and the claim is in a reference doc. The test is owed once the seam, hardware or
harness named in the last column exists.

| Location | Claim | What blocks a test |
| --- | --- | --- |
| `CMakeLists.txt:198-201` | One CUDA runtime per process | Needs a GPU-linked probe of the loaded libcudart; no harness |
| `include/control_loop/control_loop_interface.hpp` (forwarders) | Forwarders are thread-safe | No sanitizer build option in CMake for a TSan test |
| `firmware/mr_stabs_mk2/include/espnow_link.h` (substitution gate) | Gate lives in `main.cpp` | Extract the gate into a pure function first |
| `firmware/mr_stabs_mk2/include/espnow_link.h` (pairing) | `pair()` uses NVS | NVS does not run natively |
| `include/publisher/foxglove_publisher.hpp` (destroy while publishing) | Member order makes destroy-while-publishing clean | Needs an ASan build for a `DestroyWhilePublishingIsClean` test |
| `include/mcap_recorder/mcap_recorder.hpp` (start_file_locked, mutex_) | Lock held / members guarded | No Clang thread-safety annotations or -Wthread-safety in the build |
| `include/rgbd_camera/video_encoder.hpp`, `zed_rgbd_camera.hpp` (lock discipline) | Members guarded by their mutex | No thread-safety annotations; GCC with unannotated std::mutex |
| `auto_battlebot/perception/trt_yolo.py` (module docstring) | Python path matches the C++ detector | Parity needs a C++ golden output from a TensorRT engine and a frame |
| `src/rgbd_camera/video_encoder.cpp:125` | `stopping_` read under lock | A test would be a nondeterministic timing race |
| `src/rgbd_camera/video_playback_camera.cpp:345` | Playback pacing uses sleep_until | `PacingHoldsTheRecordedRateWhilePipelineWorkRuns` still passes with a fixed sleep_for; the test does not guard pacing |
| `src/rgbd_camera/zed_device.cpp` (3), `zed_one_rgb_camera.cpp` (7), `zed_rgbd_camera.cpp` (4) | ZED camera behavior | No ZED SDK seam or hardware-free test; needs `sl::Camera` |
| `src/runner.cpp:68-69, :614-615` | 30 s re-open signal safety; static gate reset on field re-init | No Runner test file; Runner needs its whole dependency set |
| `src/tensorrt_inference/trt_engine.cpp:63-68, 91, 109-110, 150` | Stream reader and engine load details | `FileStreamReader` is in an anonymous namespace and a test needs a GPU and a real engine |
| `src/transmitter/esp32_diagnostics_opentx_transmitter.cpp:37` | Destruction order | Needs a serial channel |
| `src/transmitter/espnow_transmitter.cpp:345-346` | Dongle reopen is rate limited | Removing the rate limit leaves all 18 EspNow/Composite tests passing; needs a test that counts reopen attempts |
| `src/transmitter/opentx_transmitter.cpp` (5 rows) | OpenTX device handling | Device found by USB VID/PID; no test seam |
| `src/viz/nvjpg_jpeg_encoder.cpp:66-68` | Full-range YCrCb conversion | Only exercised on the Jetson NVJPG path |
| `src/viz_relay/main.cpp` (3 rows: whole-frame drops, same-poll reconnect, second app refused) | Relay socket handling | `main.cpp` is a monolithic executable with no test harness |
| `tests/esp32_diagnostics/test_esp32_wifi_telemetry.cpp:119-123` | Fake server mirrors firmware `handle_telemetry` | A shared fixture both sides test against needs the firmware build |
| `tests/test_kalman_motion_estimator.cpp` (`make_our_pose_measurement`, `make_opponent_pose_measurement`) | Helpers mark heading as measured | A `heading_measured` field on `RobotDescription` touches the tracker, the estimator and scene adapter |
| `web/src/lib/connection.ts:1-4` | Subscriptions survive reconnects | web/ has no test runner |

## Blocked on a decision or hardware

Rows a batch could not settle. Each needs the action in the last column first.

| Location | Needs |
| --- | --- |
| `auto_battlebot/mujoco_sim/onshape_export.py:1` | Decide whether purchased parts get real mass in the Onshape part studio, or export mass per part through the Onshape API; needs the Onshape exporter |
| `auto_battlebot/rl/lockstep_world.py:130` | Add `episode_s: 20.0` to the gitignored `playground/rl/out/scenarios.json`, `scenarios_fit.json` and `old_opponents/scenarios_pre_house_radius.json`, then make the key required and drop `LEGACY_SCENARIO_EPISODE_S` and `LEGACY_ROW_EPISODE_S` (`playground/rl/evaluate_nav_policy.py`) |
| `include/mask_model/fixed_mask_model.hpp:8` | Replace `FixedMaskModel` with `FieldFilterInterface::requires_mask()` (false for `FixedFieldFilter`), gate the Runner on it, switch the three sim TOMLs to `NoopMaskModel`. Needs `docs/reference/simulation/kinematic_sim_config.md`, which has uncommitted edits |
| `include/rl_policy/policy_interface.hpp:14` | Decide whether navigation outputs a `NormalizedCommand` type; it changes every navigation implementation |
| `include/robot_descriptions_cache.hpp:9` (`Resolved` lifetime) | Decide between `[[clang::lifetimebound]]` (needs a Clang build) and returning `Resolved` by value with caller rework |
| `include/rgbd_camera/v4l2_rgb_camera.hpp` (`dequeue_newest`) | The e-CAM25 is defunct, so `V4l2RgbCamera` has no camera to check against. Decide whether to delete the V4L2 camera path |
| `tests/python/test_tag_pose_smoother.py:578` | Rejected fixes sit at t=2.987, 5.587 and 7.387 s; only 5.587 is at the blackout edge. Fixing it changes smoother gating or the IMU sigma, a runtime behaviour change: decide whether to retune |
| `firmware/mr_stabs_mk2/include/wifi_manager.h:103` | Thread ownership in `wifi_manager.h` needs Clang thread-safety annotations, and the ESP32 toolchain is GCC. Decide whether to keep it as a documented invariant |
| `training/model_eval/score_size_ladder.sh:97-98` | Needs the `cage_high_x50_conf044/d40000` eval set (not in the main checkout), or confirmation that `nhrl_cage_high_eval` replaces it; that set has `reviewed: []` |
| `hardware/cage_pack/enclosure.py:221` | The failure is in Onshape, not OCC: with the constant at 0 the build123d enclosure is valid. Settle with one Onshape push of the enclosure at 0 (costs Onshape API calls from the yearly budget) |
| `hardware/cage_pack/enclosure.py:295` | The failure is in Onshape, not OCC: with the constant at 0 the build123d enclosure is valid. Settle with one Onshape push of the enclosure at 0 (costs Onshape API calls from the yearly budget) |
| `src/navigation/rl_policy_navigation.cpp:189` | Zero-command path is tested; the coasted spin and motion-blur claim needs a recording. Decide between decaying coasted yaw rate in the robot filter and training the policy on observation age |
| `hardware/cage_pack/bom_data.py:263` | Needs a change in `~/build123d-skill`: a spec key (e.g. `mcmaster_bom`) that sets part_number and vendor without loading the STEP (skill `assembly.py:304-307`) |
| `hardware/zed_box_mini_mount/assembly_page.py:62` | Needs a test of GLB handling on the artifact host, which means an upload; decide whether to allow one |
| `src/rgbd_camera/v4l2_rgb_camera.cpp` (5 rows: encoder fatal, monotonic stamps, pinned allocator, camera_info header, strictly increasing stamps) | Pending the decision on deleting `V4l2RgbCamera` (the e-CAM25 was its only camera) |
| `src/rgbd_camera/v4l2_rgb_camera.cpp:281` | Pending the decision on deleting `V4l2RgbCamera` (the e-CAM25 was its only camera) |
| `src/rgbd_camera/v4l2_rgb_camera.cpp:319` | Pending the decision on deleting `V4l2RgbCamera` (the e-CAM25 was its only camera) |
| `src/rgbd_camera/v4l2_rgb_camera.cpp:364` | Pending the decision on deleting `V4l2RgbCamera` (the e-CAM25 was its only camera) |
| `src/rgbd_camera/v4l2_rgb_camera.cpp:378` | Pending the decision on deleting `V4l2RgbCamera` (the e-CAM25 was its only camera) |
| `src/rgbd_camera/v4l2_rgb_camera.cpp:457` | Pending the decision on deleting `V4l2RgbCamera` (the e-CAM25 was its only camera) |

## Follow-up: purge the relocated comments from docs

Do this after the hacks above are fixed or pinned by tests. Until then the docs below are the only
record of why each workaround exists, and the ledger rows point at them.

The purge moved comments into docs instead of deleting them. A 2026-11-20 sweep classified the 145
docs the purge commits created or appended to:

| Kind | Docs | Action |
| --- | --- | --- |
| Measurement | 25 | Keep |
| Reference | 18 | Keep |
| Path or link edits only | 13 | Nothing |
| Relocated comments, whole doc | 14 | Delete the doc, after writing any test it names |
| Relocated comments in sections | 75 | Delete those sections, keep the rest |

Relocated comments go stale faster than the comments did, because nothing next to the code
reminds an editor they exist. Five days after the move:

- Every "Line N, above X" anchor in `docs/hardware/` is wrong (`CP10_PITCH` moved from 38 to 20)
- `kinematic_sim_design.md` and `shared_hazards_file.md` cite `simulation/*.py`, now
  `auto_battlebot/kinematic_sim/`
- `opentx_transmitter.md` cites `kChannelTimeout`, now `kChannelStreamStallTimeout`
- `2026-09-22_zed_frame_rotation.md` defends the "buffer moved" log in `ZedDevice::retrieve`
  after the root cause was fixed
- The basement scene doc warns against a seek in `export_svo_depth.py`, which has none
- `zed_box_mini_mount.md` lost its field names in the move and is unreadable

### What to remove

- `## Workarounds` and `## Workarounds in tests` sections: 34 docs, 37 with "Unverified", "Not
  verified" or "not reproduced" entries. Each entry goes once its ledger row is closed
- Sections keyed to a symbol, file, or line: `### <file>.py`, `<function>:`, "Line N, above X",
  "Beside `...`". The largest are `docs/reference/calibration/sysid_modules.md` (about 960 lines,
  130 headings) and `docs/reference/recording/foxglove_recording_format.md` (about 368 lines)
- Bullets that restate a test: `docs/reference/control/hazards.md` repeats about 14 invariants
  already pinned by `tests/hazards/*` and `HazardBarrierTest.*`
- History ("the old brake", "previously", "when the comment was written") and plan checkpoints
- Implementation notes appended to dated experiments: `opponent_filter_evaluation.md`,
  `mujoco_mr_stabs_model_spike.md`, `rl_navigation_report.md`. Keep their results

### What to keep or move

- Measurements stay in `docs/experiments/`. Rows missing a date, box, or recording get one or
  lose the number. Several carry the purge date (2026-11-15) instead of the measurement date
- How-it-works prose that sits in `docs/experiments/` moves to `docs/reference/` (16 docs, mostly
  the `*_scene_pipeline`, sysid smoother, and apriltag docs)
- Fitted-value provenance in reference docs moves to a dated experiment doc:
  `camera_calibrations.md`, `robot_plant_files.md`, the sweep numbers in `synthgen_cage_specs.md`

### Tunables defended in prose

These are justified in docs instead of living in a config struct. Move each to a defaulted config
field, then delete the prose:

- `kExcessGain = 1.5` in `src/navigation/hazard_avoidance.cpp`, duplicated in
  `tests/hazards/test_hazard_geometry.cpp`
- `kMaxPushOutPasses = 6` in `src/hazards/hazard_geometry.cpp`
- `kChannelStreamStallTimeout` in `src/transmitter/opentx_transmitter.cpp`
- `GATE_MARGIN` and `TAG_DROPOUT_MIN_GAP_NS` in the sysid tag smoother path

### Docs that are relocated comments start to finish

- `docs/experiments/control/2026-11-15_kinematic_sim_design.md`
- `docs/experiments/control/2026-11-15_rl_policy_bundle.md`
- `docs/experiments/control/hazard_avoidance/2026-11-15_shared_hazards_file.md`
- `docs/experiments/control/nav_eval/2026-10-25_reverse_freeze_repro.md`
- `docs/experiments/perception/2026-11-15_field_corner_naming.md`
- `docs/experiments/perception/2026-11-15_free_roam_field_mask.md`
- `docs/experiments/perception/2026-11-15_parallel_model_batch.md`
- `docs/experiments/perception/rgb_homography/2026-11-15_field_transform_comparison.md`
- `docs/experiments/platform/2026-09-22_zed_frame_rotation.md`
- `docs/experiments/platform/2026-11-15_battery_monitor_threading.md`
- `docs/experiments/platform/2026-11-15_jpeg_encoder_selection.md`
- `docs/hardware/README.md`
- `docs/reference/perception/gpu_letterbox_memory.md`
- `docs/reference/robot_link/opentx_transmitter.md`

### Docs with relocated-comment sections

- `docs/experiments/control/2026-11-15_control_loop_threading.md`
- `docs/experiments/control/2026-11-15_rl_policy_hazard_barrier.md`
- `docs/experiments/control/kalman_filter/jig_calibration_provenance_2026-08-20.md`
- `docs/experiments/control/kalman_filter/opponent_filter_evaluation.md`
- `docs/experiments/control/mujoco_mr_stabs_model_spike.md`
- `docs/experiments/control/nav_eval/2026-11-15_forward_step_test.md`
- `docs/experiments/control/nav_eval/2026-11-15_rl_nav_metrics.md`
- `docs/experiments/control/rl_navigation_report.md`
- `docs/experiments/hardware/2026-09-25_mr_stabs_mk2_mass_properties.md`
- `docs/experiments/hardware/2026-11-08_cage_pack_bom_prices.md`
- `docs/experiments/hardware/2026-11-12_cage_pack_shock_model.md`
- `docs/experiments/hardware/2026-11-15_drivetrain_bldc_conversions.md`
- `docs/experiments/perception/2026-09-04_depth_mode_coverage.md`
- `docs/experiments/perception/2026-11-15_basement_scene_pipeline.md`
- `docs/experiments/perception/2026-11-15_cage_high_annotation.md`
- `docs/experiments/perception/2026-11-15_cage_high_scene_pipeline.md`
- `docs/experiments/perception/2026-11-15_homography_field_filter.md`
- `docs/experiments/perception/2026-11-15_massd_scene_pipeline.md`
- `docs/experiments/perception/brettzone_cage_high_recordings.md`
- `docs/experiments/platform/2026-09_camera_following_clock.md`
- `docs/experiments/sysid/2026-08-23_process_noise_fit.md`
- `docs/experiments/sysid/2026-10-15_mujoco_open_loop_heading_hold.md`
- `docs/experiments/sysid/2026-10-15_mujoco_reverse_window_stratification.md`
- `docs/experiments/sysid/2026-10-15_sysid_yolo_dropout_fill.md`
- `docs/experiments/sysid/2026-10-25_mrs_buff_mk3_model.md`
- `docs/experiments/sysid/2026-10-25_sysid_tag_smoother.md`
- `docs/experiments/sysid/2026-11-15_apriltag_ground_truth.md`
- `docs/hardware/cage_pack.md`
- `docs/hardware/mr_stabs_mk2.md`
- `docs/hardware/mr_stabs_mk2_pcb.md`
- `docs/hardware/tx16s_hr10.md`
- `docs/hardware/zed_box_mini_mount.md`
- `docs/reference/calibration/plant_fit_modules.md`
- `docs/reference/calibration/sysid_modules.md`
- `docs/reference/calibration/velocity_jig_config.md`
- `docs/reference/config/profiles.md`
- `docs/reference/config/robot_plant_files.md`
- `docs/reference/control/control_loop.md`
- `docs/reference/control/hazards.md`
- `docs/reference/control/plant_backed_control.md`
- `docs/reference/control/rl_opponents.md`
- `docs/reference/control/rl_policy_bundle.md`
- `docs/reference/control/rl_training.md`
- `docs/reference/eval/eval_code.md`
- `docs/reference/perception/camera_calibrations.md`
- `docs/reference/perception/field_pose.md`
- `docs/reference/perception/robot_filter_pipeline.md`
- `docs/reference/perception/trt_engine.md`
- `docs/reference/perception/trt_yolo.md`
- `docs/reference/recording/foxglove_recording_format.md`
- `docs/reference/recording/svo_recording.md`
- `docs/reference/recording/video_recording.md`
- `docs/reference/remote/protocol.md`
- `docs/reference/robot_link/esc_drive.md`
- `docs/reference/robot_link/esp32_wifi_telemetry.md`
- `docs/reference/robot_link/espnow_link.md`
- `docs/reference/robot_link/firmware_build.md`
- `docs/reference/robot_link/motor_guard.md`
- `docs/reference/robot_link/tuning.md`
- `docs/reference/simulation/kinematic_sim_config.md`
- `docs/reference/tools/gpu_queue.md`
- `docs/reference/tools/kinematic_sim_viewer.md`
- `docs/reference/tools/logo_splash_animation.md`
- `docs/reference/tools/python_packaging.md`
- `docs/reference/tools/synthgen_cage_specs.md`
- `docs/reference/tools/synthgen_render.md`
- `docs/reference/tools/synthgen_scene_contracts.md`
- `docs/reference/tools/yolo_training.md`
- `docs/setup/build_notes.md`
- `docs/setup/claude_code_setup.md`
- `docs/setup/dashboard_kiosk.md`
- `docs/setup/docker_playback.md`
- `docs/setup/install_notes.md`
- `docs/setup/jetson_setup.md`
- `docs/setup/zed_box_mini.md`

### Rule for next time

Add to the Comments section of `CLAUDE.md`: a comment moved into a doc is still a comment. Docs
hold measurements and how the code works now, never a defense of a specific line. No
`Workarounds` sections.
