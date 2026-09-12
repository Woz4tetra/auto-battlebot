# Migrating to an RGB-only camera

Status: **implemented, apart from what needs the hardware** (2026-09-09). See "What landed" at the
bottom. Companion to `docs/plans/field_transform_cpp_migration_2026-09-09.md`,
which removes the depth dependency from field fitting. This document covers the rest: getting
`e-CAM25_CUONX_H01R1` frames into `CameraData`, replacing SVO recording and playback, and
everything that breaks when `CameraData::depth` is empty.

## The camera

From e-con's page for `e-CAM25_CUONX`:

| | value |
| --- | --- |
| Sensor | onsemi AR0234CS, 1/2.6", 3 um pixels, global shutter |
| Array | 1920x1200 |
| Rates | 1280x720 @ 120 fps, 1920x1080 @ 70 fps, 1920x1200 @ 60 fps |
| Interface | MIPI CSI-2, 15 cm FPC, plugs into the Orin NX / Orin Nano dev kit |
| Output | UYVY, uncompressed |
| Driver | V4L2 (e-con out-of-tree kernel module), JetPack 6.2 / L4T 36.4.3 |
| Lens | M12 holder; supplied lens is 104.6 deg horizontal, 61.6 deg vertical, 128.2 deg diagonal |

Two properties drive the software design. It is V4L2, not an SDK, so there is no positional
tracking, no calibration blob, and no recording facility to inherit. And it is a wide M12 lens,
not a ZED rectified stream, so distortion is ours to model.

## What changes

| | today | after |
| --- | --- | --- |
| Capture | `ZedRgbdCamera` -> `sl::Camera::grab` | `V4l2RgbCamera` -> `VIDIOC_DQBUF` |
| Frame | rectified BGR + float depth + visodom pose | rectified BGR, empty depth, identity pose |
| Intrinsics | read from the ZED at open | read from a calibration file |
| Recording | `SvoRecorder` -> `.svo2`, plus a separate pipeline `.mcap` | H.264 on `/camera/video` in the one `.mcap` |
| Playback | `ZedSvoPlaybackCamera` | `VideoPlaybackCamera`, reading three channels of the recording |
| Field fit | `PointCloudFieldFilter` (needs depth) | `HomographyFieldFilter`, `FiducialFieldFilter`, or a per-cage calibration |
| Height gate | measures relief from depth | abstains; `StaticDetectionGate` takes over |
| Build | `find_package(ZED 5 REQUIRED)` | `BUILD_WITH_ZED`, default ON, off on the new box |
| OpenCV | apt 4.5.4 / 4.6.0, Jetson 4.10.0 | 4.8 from source everywhere, past the aruco API break |

## 1. Settle the field transform first, because the mat corners are marginal

The companion plan extracts four mat corners from the field mask and solves a homography against
the known mat size. Mounted the way NHRL mounts its own cage cameras, about 1.2 m off the floor
and tilted down, whether we see all four is a close call that the spec sheet cannot settle.

Coverage of the action is not the question. NHRL shoots with an iPhone Pro at 1x, a 24 mm
equivalent giving 71.6 degrees horizontal and, cropped to 16:9 video, about 44 degrees vertical.
This camera is wider on both axes. Anything their cameras frame, ours frames with room left.

The question is the mat boundary, and it turns on focal length rather than on the quoted field of
view. Backing f out of each of e-con's three numbers at 1920x1200 gives three different answers:

| from | half-angle | implied f |
| --- | --- | --- |
| 104.6 deg horizontal | 52.3 deg | 960 / 1.295 = 741 px |
| 61.6 deg vertical | 30.8 deg | 600 / 0.596 = 1006 px |
| 128.2 deg diagonal | 64.1 deg | 1132 / 2.058 = 550 px |

No pinhole model fits all three, which is what a wide M12 lens looks like: the quoted field is
measured optical field, inflated by barrel distortion. The vertical number sits closest to the
axis and is the least distorted, so f is near 1006 px and the rectilinear horizontal field after
undistortion is 2 atan(960/1006) = 87 degrees. That 87 degrees, not 104.6, is what survives
`initUndistortRectifyMap` and what the geometry has to be checked against.

Against 87 degrees, with the camera at height h and standoff d behind the wall plane, the near
mat corners clear the horizontal edge of frame only when `sqrt(d^2 + h^2) > 1.2192 / tan(43.5 deg)`,
which is 1.278 m. At 1.2 m height that wants 0.44 m of standoff; at a 0.1 m standoff it wants
1.28 m of height. Either way we are within centimetres of the boundary, and the vertical
constraint is tighter still: the downtilt window that holds the near corners and the far edge in
frame at once is a few degrees wide. That is not a margin to machine a fixture against, least of
all from spec-sheet numbers that do not agree with each other.

So plan for a partially visible outline. Fitting lines rather than corners costs little and
removes the dependency on a measurement we cannot make until the lens is calibrated.

Three fixes, in the order I would try them.

**a. Fit lines, intersect to virtual corners.** With `K` known, the plane-induced homography
`H = K[r1 r2 t]` has six degrees of freedom. Two finite corners give four constraints and the
vanishing point of the two side edges gives two more. Six for six: three visible boundary lines
determine the pose exactly, and the missing corners fall out as intersections outside the image.
`refine_quad_by_edges` already fits lines by total least squares. The changes are to drop contour
points lying within `border_margin_px` of the frame before fitting, to fit only the sides that
have surviving support, and to let `cv::findHomography` take corners with coordinates off the
sensor. Replace the border-contact guard with a per-side support count: reject when fewer than
three sides have enough points.

**b. Calibrate each cage once and stop fitting at match time.** This is what the quick-connect
fixture is worth. Seat the box on the suction cup mount, run the line-fit above against a clean
empty-cage frame, and write `tf_camera_from_fieldcenter` plus the mat size into
`config/cages/<venue>_<cage>.toml`. At match time a new `CalibratedFieldFilter` loads it and
publishes it unchanged. `FixedFieldFilter` in `include/field_filter/fixed_field_filter.hpp` is
close but takes its transform from `tf_visodom_from_camera`, which is identity here, so it needs
a configured transform instead of that argument.

I would ship b as the match-day path and keep a as the tool that produces the file and as a
seating check. Compare the live line fit against the stored calibration each time the app starts
and warn when corner reprojection exceeds a few pixels: that catches a fixture that did not seat
or a cage that got bumped, which is the failure mode a static camera cannot otherwise detect.

**c. Skip the mat outline entirely and fit to a fiducial board.** Section 2. This is the strongest
option at an unsurveyed venue, since it depends on nothing we have to segment or assume. It needs
the board allowed inside the cage between matches, which is a question for NHRL, not for us.

Either way the field transform must land before the camera does. The RGB camera cannot run with
`_common.toml`'s current `PointCloudFieldFilter`.

## 2. Fiducial board mode

The floor grid at `playground/calibration/print/floor_grid.pdf` removes the dependency on seeing
the mat boundary at all. Place the board at a known corner, detect it once at field init, and the
camera pose falls out of a PnP solve over 60 corner correspondences. Config supplies the field
extent relative to that corner. Nothing in the fit cares whether the mat outline is clipped,
cluttered, or segmented well, which makes this the mode I would run at a venue we have not
surveyed.

The board is the manufactured 3x5 AprilTag 36h11 grid: 65 mm markers, 15 mm gaps, ids 160 to 174,
225 mm by 385 mm overall. `auto_battlebot/calibration/apriltag/apriltag_detect.py` already solves
exactly this problem for the drivetrain jig, and `make_floor_board` and `solve_floor_extrinsic`
there are the reference implementation to port.

### Where the board goes, and the numbers that decide it

A 65 mm marker needs roughly 18 px of edge to decode and about 30 px to be comfortable, per
`make_print_tags.py`'s own thresholds. At the estimated f of 1006 px, from a 1.2 m mount with a
0.15 m standoff, laid flat on the floor:

| placement | range | marker edge | foreshortened | verdict |
| --- | --- | --- | --- | --- |
| near mat corner | 1.72 m | 38 px | 27 px at 44 deg | works |
| far mat corner | 3.10 m | 21 px | 8 px at 23 deg | will not decode |

Foreshortening is what kills the far placement. A board flat on the floor viewed 23 degrees above
the plane compresses its depth axis by `sin(23 deg)`, and scaling the markers does not rescue it:
150 mm markers at the far corner still only reach 19 px, on a board that would be 500 mm by
850 mm.

So put it at a **near** corner. That composes well, because the near corners are precisely the ones
section 1 shows the camera cannot resolve as mat corners. The board covers the weak spot.

A third option is worth keeping: mount the board **vertically on the far wall**. Face-on at 3.1 m
there is no foreshortening, so 65 mm markers give a flat 21 px and 100 mm markers give 32 px. That
needs the board-to-floor offset in config, which the offset and orientation fields below already
carry, and it survives a mat that is scuffed or a floor that is not clean.

### `FiducialFieldFilter`

A third `FieldFilterInterface` implementation beside `HomographyFieldFilter` and
`CalibratedFieldFilter`. `compute_field` ignores the field mask entirely and works off
`camera_data.rgb`.

1. Detect 36h11 markers in the frame.
2. Keep only ids belonging to the configured board, and require at least `min_markers`.
3. Pair image corners to board object points and stack correspondences across
   `accumulate_frames` frames, as `solve_floor_extrinsic` does, so detection noise averages out
   before the pose latches.
4. `cv::solvePnP` over the stack gives `R_bc, t_bc`, the board frame in camera coordinates.
5. Compose the configured board-to-corner and corner-to-center transforms to get
   `tf_camera_from_fieldcenter`, and set `size` from the configured field extent.

Step 5 is the part config owns. The rest of the pipeline works in field-center coordinates, so the
filter converts corner to centre by translating half the field extent along each axis, with the
sign chosen by which corner the board marks.

```cpp
struct FiducialFieldFilterConfiguration : public FieldFilterConfiguration {
    // Board geometry. Defaults match the manufactured board.
    int board_cols = 3;
    int board_rows = 5;
    double marker_size = 0.065;      // metres, printed edge
    double marker_separation = 0.015;
    int first_marker_id = 160;

    // Where the board sits. corner names which field corner it marks; the offsets are the
    // board origin measured from that corner, in the field frame, because the board cannot
    // physically sit in the corner itself.
    FieldCorner corner = FieldCorner::NEG_X_NEG_Y;
    double board_offset_x = 0.0;
    double board_offset_y = 0.0;
    double board_offset_z = 0.0;     // non-zero for a wall-mounted board
    double board_yaw_deg = 0.0;
    double board_pitch_deg = 0.0;    // 90 for a board hung flat on a wall

    // Field extent measured from that corner.
    double field_size_x = 2.35;
    double field_size_y = 2.35;

    int min_markers = 4;
    int accumulate_frames = 10;
    double max_reprojection_error_px = 3.0;

    FiducialFieldFilterConfiguration() { type = "FiducialFieldFilter"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD(board_cols)
        PARSE_FIELD(board_rows)
        PARSE_FIELD_DOUBLE(marker_size)
        PARSE_FIELD_DOUBLE(marker_separation)
        PARSE_FIELD(first_marker_id)
        PARSE_ENUM(corner, FieldCorner)
        PARSE_FIELD_DOUBLE(board_offset_x)
        PARSE_FIELD_DOUBLE(board_offset_y)
        PARSE_FIELD_DOUBLE(board_offset_z)
        PARSE_FIELD_DOUBLE(board_yaw_deg)
        PARSE_FIELD_DOUBLE(board_pitch_deg)
        PARSE_FIELD_DOUBLE(field_size_x)
        PARSE_FIELD_DOUBLE(field_size_y)
        PARSE_FIELD(min_markers)
        PARSE_FIELD(accumulate_frames)
        PARSE_FIELD_DOUBLE(max_reprojection_error_px)
    )
    // clang-format on
};
```

`FieldCorner` is new, in `include/enums/field_corner.hpp` and included from `include/enums.hpp`:

```cpp
enum class FieldCorner { NEG_X_NEG_Y, NEG_X_POS_Y, POS_X_NEG_Y, POS_X_POS_Y };
```

Naming the corner by the signs of its field-frame coordinates rather than by compass direction
keeps it readable next to the corner-to-centre translation, which is exactly those two signs times
half the field extent.

### Two things that will bite

**Marker ids run right to left.** `make_floor_board` carries a hard-won comment: the manufactured
board numbers markers right to left within each row, `cv::aruco::GridBoard` fills them left to
right, and pairing them the wrong way round gives a reflection rather than a rotation. The measured
cost was 440 px of reprojection error against 16 px when correct. The C++ port must reverse each
row the same way, and a unit test should assert the id at a known grid position. `make_robot_tag_3d.py` shipped exactly this
failure once: its mesh was a vertical mirror of the bit grid, and `verify()` rasterized the bit
array rather than the mesh, so it passed while the printed tag could not decode at all. A board
that decodes perfectly and is mirrored is the worse version of that, because it looks like a
working system until the pose is wrong.

**OpenCV's aruco API moved at 4.7.** Section 2a covers it. It is a toolchain change rather than a
code one, so it is written up separately.

### Why reprojection error is a real guard here

The companion plan notes that reprojection error is useless for the four-corner homography, which
fits whatever corners it is handed at 0.0 px. This mode is the opposite case: 15 markers give 60
point correspondences against 6 pose degrees of freedom, so the residual actually measures
something. Reject above `max_reprojection_error_px` and log it at warn level. It catches a
mis-measured `marker_size`, a board printed at "fit to page" rather than 100%, a mirrored id
mapping, and a board that was not flat.

### What it costs and what it buys

The board has to be placed and measured before each event, and removed before the match starts.
That is the price. What it buys is a field transform that does not depend on segmenting the mat,
on the mat's size being what we think it is, or on the camera seeing corners it may not see.

It also replays. Detection runs off `camera_data.rgb`, so a recording whose first frames contain
the board carries everything needed to re-derive the field transform later. That is worth building
into the event routine: record ten seconds with the board down before pulling it, every time.

I would run this mode at any venue we have not surveyed, and fall back to the per-cage calibration
file of section 1b once a cage has been measured and the fixture is repeatable.

## 2a. Upgrade desktop OpenCV to 4.8

`cv::aruco` moved from the contrib module `opencv_aruco` into core `opencv_objdetect` at 4.7, and
the API changed with it: free-function `detectMarkers` and `GridBoard::create` became the
`ArucoDetector` class, a `GridBoard` constructor, and `Board::matchImagePoints`. Every call in
`apriltag_detect.py` that section 2 ports is on the new side of that line.

Today the tree spans both sides, and by more than I first credited:

| platform | OpenCV | source |
| --- | --- | --- |
| Ubuntu 22 desktop | 4.5.4 | apt `libopencv-dev`, jammy |
| Ubuntu 24 desktop | 4.6.0 | apt `libopencv-dev`, noble |
| Jetson | 4.10.0 | `install/install_opencv_jetson.sh`, built from source |
| Jetson, if JetPack already installed one | 4.8.0 | the script's early exit fires |

Four possible versions across two supported desktops and one deploy target, straddling the API
break. Pin the desktop to **4.8.0 from source** and every platform lands at or above 4.7, so the
fiducial port is a straight transcription of `apriltag_detect.py` with no `CV_VERSION_MAJOR` or
`CV_VERSION_MINOR` branching anywhere. That is the whole reason to do this.

**The work.** `install/install_opencv_jetson.sh` already does a from-source build with resume
support and is parameterized on version, build folder, CUDA arch, and Python version. Generalize it
to `install/install_opencv.sh` and call it from `scripts/install_ubuntu_22.sh` and
`scripts/install_ubuntu_24.sh`, then drop `libopencv-dev` from `install/ubuntu_22_packages.txt` and
`install/ubuntu_24_packages.txt`. Three changes to the script itself:

- Make CUDA optional and leave it off on the desktop. `WITH_CUDA`/`WITH_CUDNN` are there for the
  Jetson; nothing on the desktop calls a `cv::cuda` entry point, and TensorRT does the inference.
  Off, the build is much shorter.
- Set `BUILD_opencv_python3=OFF` on the desktop. The venv gets `cv2` from pip at 4.13, and a
  `/usr/local` `cv2` would shadow it.
- Drop `OPENCV_EXTRA_MODULES_PATH`. Contrib existed for aruco; at 4.7+ aruco is core, and
  `find_package(OpenCV ...)` at `CMakeLists.txt:110` asks only for core modules.

**Assert the version in CMake** so a wrong pick fails at configure instead of at the first aruco
call:

```cmake
find_package(OpenCV 4.8 REQUIRED COMPONENTS core imgproc highgui dnn objdetect calib3d)
```

`objdetect` for aruco and `calib3d` for `solvePnP`, neither of which is in the current component
list even though `calib3d` is already used transitively.

**Three traps.**

- `OpenCV_DIR` is cached. `build/CMakeCache.txt` currently pins
  `/usr/lib/x86_64-linux-gnu/cmake/opencv4`, so an existing build directory keeps resolving 4.6.0
  after the upgrade and the failure looks like the new install did not take. Run
  `./scripts/clean_build.sh` as part of the upgrade.
- The script's early exit tests `pkg-config --exists opencv4`, which is true on any machine that
  ever had apt OpenCV. As written the desktop upgrade would silently do nothing. It has to compare
  the found version against the requested one and rebuild when it is lower, which also fixes the
  Jetson case where JetPack's 4.8.0 currently pre-empts the 4.10.0 build.
- Leave the apt runtime libraries installed. Other packages link them and removing
  `libopencv-dev` would cascade. Installing to `/usr/local`, which CMake searches first, plus the
  version assertion above, is enough.

**Decided: 4.10.0 across the fleet**, rather than dropping the Jetson to 4.8.0. Both are past the
API break so it changes nothing for section 2, and taking the version the Jetson already built
means the deploy target does not move.

## 3. What else breaks without depth

`grep` puts depth consumers outside `src/rgbd_camera/` in four places.

**`KeypointHeightGate`, `src/runner.cpp:436-439`.** It abstains on an empty depth image, so nothing
crashes, but two behaviors quietly stop. Rejection stops: memory of the 2026-08-29 MassD replay has
it removing 91% of arena-logo detections while keeping 90% of opponents, and that suppression goes
away. Measurement stops too, so every keypoint keeps NaN height and projection falls back to the
`keypoint_height_meters` constant, which is the pre-gate behavior and is survivable.

The replacement is `StaticDetectionGate`, already implemented in
`src/keypoint_filter/static_gate.cpp` and currently off by default because the height gate beat it
everywhere. Without depth it is the only candidate left. Turn it on, re-run the MassD and AER
recordings through it, and record the delta honestly rather than assuming parity. Expect it to be
worse; 91% of MassD false positives sit in fixed field-frame clusters, which is what the static
gate is built for, so it should recover most of the win.

**`PointCloudFieldFilter`.** Covered above.

**`to_field_point_cloud`, `src/foxglove_adapters/scene.cpp:138`.** Returns `nullopt` on an empty
cloud and `foxglove_publisher.cpp:147` already handles that. `/field_points` simply stops
publishing. No change needed.

**`SimRgbdCamera` via `src/simulation/sim_connection.cpp:201`.** Simulation still produces depth.
Leave it alone; the sim path is not affected by the hardware change.

Two build consequences follow. PCL is used only by `PointCloudFieldFilter` and its debug view, so
`find_package(PCL REQUIRED COMPONENTS common sample_consensus)` can go once that filter does.
And `data_structures/point_cloud.hpp` becomes dead.

## 4. Keep the ZED path, do not delete it

`find_package(ZED 5 REQUIRED)` becomes:

```cmake
option(BUILD_WITH_ZED "Build the ZED camera and SVO playback backends" ON)
```

with the four ZED translation units, the `ZED`/`CUDA` link entries at `CMakeLists.txt:249`, the
debug-flag override at `CMakeLists.txt:287`, and the `REGISTER_CONFIG` lines for `ZedRgbdCamera`
and `ZedSvoPlaybackCamera` all guarded. The glob at `CMakeLists.txt:158` is `GLOB_RECURSE` over
`src/**/*.cpp`, so guarding means a `list(FILTER ...)` on `src/rgbd_camera/zed_*` and
`src/field_filter/point_cloud_*` rather than an `if` around an explicit list.

Every SVO in `data/svo/`, the keypoint corpora, the eval ground truth in
`training/model_eval/`, and every regression recording the `/replay` skill drives are tied to the
ZED path. Deleting it strands all of that. The Jetson image for the new box builds with
`-DBUILD_WITH_ZED=OFF` and skips the ZED SDK install; desktop keeps it ON until the RGB corpus is
large enough to retire the SVO one.

## 5. Capture: raw V4L2, not OpenCV, not GStreamer

`src/rgbd_camera/v4l2_rgb_camera.cpp` opens the node, negotiates `V4L2_PIX_FMT_UYVY` at the
configured size and rate, mmaps four buffers, and runs the same capture-thread shape
`ZedRgbdCamera` already uses: a thread calling `DQBUF`, converting, and publishing into
`latest_data_` under `data_mutex_` with `data_cv_`, and a `get()` that waits for a frame counter
change. Reusing that structure keeps `GrabHealthMonitor` and the capture-timing diagnostics
working unchanged.

Why not the alternatives. `cv::VideoCapture(CAP_V4L2)` hides the buffer count and does not expose
`v4l2_buffer.timestamp`, and we need that timestamp: it is the kernel's monotonic capture instant
and it is what `sl::TIME_REFERENCE::IMAGE` was giving us. GStreamer with `nvv4l2camerasrc` adds a
dependency and a queue of unknown depth, and queue depth is latency in a 60 ms budget. Raw V4L2
with four buffers costs us at most one frame of queueing and no new dependency.

Details that matter:

- Request `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` and convert against `CLOCK_MONOTONIC` to the
  pipeline's `double` seconds. Do not use wall clock; the existing `ManualClock` replay path
  depends on the frame stamp being the capture instant.
- Four buffers, not the driver default. More buffers means the pipeline reads staler frames when
  it falls behind. Drain to the newest queued buffer on each `get()` rather than consuming the
  oldest.
- UYVY to BGR is `cv::cvtColor(src, dst, cv::COLOR_YUV2BGR_UYVY)`. At 1920x1200 that is 4.6 MB in,
  6.9 MB out, roughly 2 to 3 ms on an Orin core. Measure it; if it lands on the critical path,
  move it onto the CUDA preprocess that already feeds the detectors.
- e-con's controls are plain V4L2 ioctls. Expose exposure, gain, and white balance in config and
  set them at open. Auto-exposure hunting between a lit arena and a dark robot is a plausible
  source of frame-to-frame detector instability, so being able to pin them matters.
- `Resolution` in `include/enums/resolution.hpp` already carries `RES_1920x1200`, `RES_1920x1080`,
  and `RES_1280x720`, so the V4L2 config reuses it rather than taking loose `width` and `height`
  integers. Add entries there if a mode we want is missing.

## 6. Calibration and rectification

The ZED handed us rectified frames and near-zero distortion coefficients. A 104.6 degree M12 lens
does not, and every downstream consumer assumes a pinhole: the homography fit, keypoint
projection, the UI marker overlay, and the trained detectors.

Rectify inside the camera implementation. Build `cv::initUndistortRectifyMap` once at open from a
calibration file and `cv::remap` each frame, publishing the rectified `K` with `D` set to zeros so
`camera_info.cpp` keeps emitting a `plumb_bob` model that happens to be trivial. Pick the alpha
for `cv::getOptimalNewCameraMatrix` so the full cage survives; at this FOV a cropping alpha will
cut exactly the mat edges the field fit needs.

Calibrate through polycarbonate. The camera sits behind a cage panel, and a flat sheet in the
optical path refracts increasingly toward the frame edges, which is where the field boundary sits.
Shoot the checkerboard through an offcut of the same material at the same standoff. If we
calibrate in free air we will fold that error into the field pose and never see it in the
reprojection residual, which is the same trap the companion plan documents for corner extraction.

Store the result at `config/cameras/<serial>.toml` with `width`, `height`, `fx`, `fy`, `cx`, `cy`,
`k1 k2 p1 p2 k3`, and a `calibration_id` string. Record `calibration_id` in the MCAP metadata
alongside `active_profile`. Record raw, unrectified frames and rectify on playback with the file
named in the recording, so a bad calibration can be redone against footage we already have.

## 7. Recording: H.264 into the main MCAP

`.svo2` is an MCAP container holding Annex-B H.264 access units, one per frame, keyed by
`log_time`. `auto_battlebot/recording/svo2.py` documents this. So the replacement is the same idea
in a schema Foxglove Studio scrubs natively, written into the recording we already produce rather
than beside it. One run, one file.

One new channel, and `/camera/image` is left exactly as it is:

| Topic | Encoding | Schema | Role |
| --- | --- | --- | --- |
| `/camera/video` | `protobuf` | `foxglove.CompressedVideo` | new. H.264, one Annex-B access unit per frame. Recorded |
| `/camera/image` | `protobuf` | `foxglove.CompressedImage` | unchanged. JPEG, live Foxglove debugging |
| `/camera/frame_meta` | `json` | `auto_battlebot.FrameMeta` | same topic, fields change below |

`third_party/foxglove/src/schemas.cpp:255` already provides `CompressedVideoChannel`, so this is a
new channel on the existing `McapRecorder`, not a new writer.

**The two paths are independent, and neither costs anything it should not.**
`include/publisher/output_channel.hpp:60` defines `has_consumers()` as
`num_subscribers() > 0 || records()`, and `_common.toml:286` keeps `/camera/image` in
`ignored_topics`, so `records()` is false and the roughly 10 ms JPEG at
`src/publisher/foxglove_publisher.cpp:88` runs only while someone is actually watching in Foxglove.
In a match with no client attached it does not run at all. That is already the behavior today; it
needs no change and it is why keeping the JPEG for debugging is free.

The H.264 path is driven by recording rather than by viewers, and it lives in the camera
implementation rather than in `FoxglovePublisher`. The camera holds the UYVY buffer the encoder
wants, section 7's encoding notes below want to skip the BGR detour, and there is no reason to
encode when nothing is being recorded. So the camera's encoder thread writes `/camera/video`
straight to the `McapRecorder` channel, and `publish_camera_data` is untouched.

**What this does for the eval tooling.** Nothing breaks and one thing gets easier. The label flow
records `/camera/image` because `config/experiments/mrs_buff_mk3_label_playback.toml` sets
`ignored_topics = []`, so `make_eval_dataset.py`, `export_labels.py`, `export_camera_transforms.py`,
`apriltag_mcap.py` and `analyze_apriltag_mcap.py` keep reading exactly what they read now. What
changes is that ordinary match recordings now carry frames at all. Previously they carried neither
JPEG nor video, which is why `scripts/combine_mcap_svo.py` exists: 34 KB whose only job is joining a
pipeline MCAP to a separate SVO by frame index. Once `mcap_io.py` decodes `/camera/video`, those
tools can point straight at a match recording and that script is deleted rather than ported.

**`FrameIdentity` loses two fields and keeps one.** `svo_frame_index` and `svo_path` existed to
join a pipeline MCAP to a separate SVO file. With one file that join does not exist. Keep
`image_stamp_ns`, which is still the original-clock stamp that relates a video frame to the
pipeline messages around it without decoding anything, and keep an ordinal as
`video_frame_index` so `start_frame` has an unambiguous meaning across a rollover boundary. Drop
`video_path`. That is a breaking change to the JSON schema text at
`include/foxglove_adapters/json_schemas.hpp:11`, which `docs/foxglove_recording_format.md` requires
stay byte-identical with `auto_battlebot/recording/mcap_write.py`, so all three move in one commit
with the old field names kept readable on the Python side.

**Rollover moves to `McapRecorder`.** This is what the sidecar was avoiding and it is the one real
cost of folding in. `SvoRecorder` owns filename generation, holding-directory eviction against
`svo_holding_dir_max_size_gb`, and rollover past `svo_max_size_gb`; all three now belong on the
MCAP, which has become the large file. Rolling means closing the writer, opening the next path, and
re-creating every channel in `channels_` against the new writer so each segment is independently
readable. `McapRecorder` already holds `channels_` and `channel_by_topic_`, so that is mechanical.
Note the app writes one MCAP per run already, keyed on start time by `make_file_path`, so rollover
only matters for a single run left going for hours.

### Encoding

H.264 through `libavcodec`, one dependency covering `h264_nvenc` on the desktop,
`h264_nvv4l2m2m` on the Jetson, `libx264` as a fallback, and the decoder playback needs anyway. At
15 Mbps that is 1.9 MB/s, about 6.8 GB per hour, and a three-minute match is about 340 MB.

Four settings that are not defaults and matter here:

- **IDR every 30 frames.** Half a second at 60 fps. It sets the seek granularity in section 8 and
  bounds how far a decoder has to run forward from a chunk boundary.
- **`max_b_frames = 0`.** B-frames reorder PTS, which would break the assumption that the nth
  `/camera/video` message is the nth captured frame. `start_frame` depends on that, and so does
  reading `log_time` as the capture instant.
- **Encode from the UYVY buffer, not from BGR.** The V4L2 buffer is already close to what the
  encoder wants; `libswscale` goes UYVY to NV12 without the detour through the BGR copy the
  detectors use. It also means the recording is pre-rectification, which section 6 wants so a
  calibration can be revised against footage we already have.
- **Encoder on its own thread, bounded queue, drop on overflow.** CLAUDE.md forbids blocking calls
  in the perception loop, and a submit-and-wait to NVENC is one. A queue of four frames and a
  dropped-frame counter in diagnostics keeps a slow encoder from turning into pipeline latency.
  Log the drops; silent frame loss in a recording is the kind of thing that is discovered months
  later against ground truth.

## 8. Playback

`VideoPlaybackCamera` reads the recording and emits `CameraData` with `depth` empty,
`tf_visodom_from_camera` identity, and `tracking_ok` true. It keeps the existing knobs under new
names: `start_frame`, `real_time_mode`, `rebase_stamps`.

**It subscribes to three channels and ignores the rest.** `/camera/video`,
`/camera/camera_info`, and `/camera/frame_meta`. Everything else in the file, the markers, the
detections, the diagnostics, the log, is the previous run's *output*, which playback exists to
regenerate. Reading it would be wrong rather than merely wasteful. MCAP indexes channels, so
filtering at the reader means those messages are never deserialized and the video is the only
payload that costs anything.

Seeking to `start_frame` uses the chunk index: find the chunk holding the last IDR at or before the
target, decode forward from there, discard until the target. Do not try to seek within a GOP.
`sl::Camera::setSVOPosition` has exactly this problem, landing on non-keyframes so the next grabs
return smeared RGB (found 2026-09-02 building `playground/render_depth_birdseye.py`), and it
applies to any inter-coded stream. With an IDR every 30 frames the worst case is 29 discarded
frames.

**Refuse to read and write the same path.** With video in the main MCAP, a replay with
`[mcap] enable = true` reads file A and records file B, and pointing the output at the input would
corrupt the source. `config/playback/_playback.toml` sets `enable = false` today, but the guard
belongs in the recorder rather than in config.

Keep the existing failure behavior for a start frame past the end. A scratch overlay that sets
`svo_file_path` but inherits `svo_start_frame` from `config/playback/_playback.toml` seeks past the
end and exits with `END OF SVO FILE REACHED` before the first heartbeat, which reads as a corrupt
recording rather than a config merge. `tests/camera/test_zed_svo_playback_camera.cpp` has a case
for it. Port that test.

`tests/camera/test.svo2` is 814 KB checked into the tree. Record an equivalent short
`tests/camera/test.mcap` and port the whole test file across: it already asserts frame size matches
`camera_info`, that repeated `get()` advances, and the deep-copy semantics at line 133. The depth
assertions at lines 87 to 90 invert, from "depth is `CV_32FC1` at image size" to "depth is empty".
Add two the SVO tests could not have: that a file whose pipeline-output channels are present is
replayed without those channels being read, and that a recording carrying only `/camera/image`
fails with a message naming the missing `/camera/video` rather than starting and producing
nothing.

### What one file opens up for the transmitter

`PlaybackTransmitter` does not replay anything today. It synthesizes an init button press after
`init_delay_seconds` and echoes back whatever navigation just sent it, because the driver's actual
stick commands were never recorded on any channel. That is why a replay cannot score our-robot
behavior against what the driver really did: the commands come from replay navigation, not from
the match.

One file makes the fix small. Record the issued commands on a `/command` channel beside the video,
and a `RecordedTransmitter` can return them in `update()` keyed on the frame being replayed, with
no join across files and no timestamp alignment. That is follow-on work rather than part of this
migration, but it is worth choosing the channel shape now, since `CommandFeedback` in
`include/data_structures/command_feedback.hpp` is already the type both sides would use.

## 9. Config and factory

New configs in `include/rgbd_camera/config.hpp`, registered in `src/rgbd_camera/config.cpp`
alongside the existing four:

```cpp
struct V4l2RgbCameraConfiguration : public RgbdCameraConfiguration {
    std::string device = "/dev/video0";          // path, so it stays a string
    Resolution camera_resolution = Resolution::RES_1920x1200;
    int camera_fps = 60;
    int buffer_count = 4;
    std::string calibration_file = "";   // config/cameras/<serial>.toml
    bool video_recording = true;
    uint64_t video_max_size_gb = 10;
    uint64_t video_holding_dir_max_size_gb = 50;
    int exposure_us = 0;                 // 0 keeps the driver default
    int gain = 0;
    V4l2RgbCameraConfiguration() { type = "V4l2RgbCamera"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_STRING(device)
        PARSE_ENUM(camera_resolution, Resolution)
        PARSE_FIELD(camera_fps)
        PARSE_FIELD(buffer_count)
        PARSE_FIELD_STRING(calibration_file)
        PARSE_FIELD_BOOL(video_recording)
        PARSE_FIELD(video_max_size_gb)
        PARSE_FIELD(video_holding_dir_max_size_gb)
        PARSE_FIELD(exposure_us)
        PARSE_FIELD(gain)
    )
    // clang-format on
};

struct VideoPlaybackCameraConfiguration : public RgbdCameraConfiguration {
    std::string video_file_path = "";
    int start_frame = 0;
    bool real_time_mode = true;
    bool rebase_stamps = true;
    std::string calibration_file = "";   // empty means use the one named in the recording
    VideoPlaybackCameraConfiguration() { type = "VideoPlaybackCamera"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_STRING(video_file_path)
        PARSE_FIELD(start_frame)
        PARSE_FIELD_BOOL(real_time_mode)
        PARSE_FIELD_BOOL(rebase_stamps)
        PARSE_FIELD_STRING(calibration_file)
    )
    // clang-format on
};
```

No pixel-format or codec knob. The camera emits UYVY and we encode H.264, so both are hard-coded,
per the same reasoning that keeps defensive config flags out of the tree: there is one correct
behavior and one consumer. If a second capture format ever turns up, that is when it becomes an
`enum class` rather than now.

`device`, `calibration_file`, and `video_file_path` stay strings because they are paths.
`camera_resolution` is an enum because it has a fixed set of valid values, per the config
convention in `CLAUDE.md`: a misspelled resolution then fails at parse with the valid values
listed rather than falling through to a runtime string compare.

Config layout follows the existing extends chain. Add `config/_orin_rgb.toml` as a platform base
next to `_jetson.toml` and `_desktop.toml`, and `config/playback/_video_playback.toml` next to
`_playback.toml`. Do not touch `_common.toml`'s camera type until the new box is the primary one.

## 10. Visual odometry goes away, and that is mostly good

A clamped camera does not move, so `tf_visodom_from_camera` is identity and
`PointCloudFieldFilter::track_field`'s rebase through `tf_visodom_from_cameraworld_` becomes a
no-op. That removes the class of bug that `scripts/fix_field_mask_frames.py` exists to
repair offline, where each field re-init redefined `field -> camera_world` and the sparse mask
topics let Foxglove project a stale mask through the new camera pose.

What it costs: if someone bumps the cage or the fixture creeps, nothing notices. That is the
seating check in section 1, and it is the reason to run the line fit at startup even when a
calibration file supplies the answer.

## 11. Detectors have to be retrained

This is outside the C++ work but it gates the whole migration, so it belongs in the schedule.

The new camera changes intrinsics, mounting geometry, viewing angle, and lens distortion at once.
Nothing in `data/engines/` transfers. Two known constraints from prior work:
The 2026-09-07 BrettZone run above found `imgsz 640` starves a 1080p detector and that `imgsz 1280`
recovers most of it. And two prior measurements say not to score one capture path's detections
against another path's labels: a desktop SVO replay warps against live Jetson output by 1.023 to
1.027 in x and 1.014 to 1.024 in y, about 13 px at 1280x720 (measured 2026-07-07 by ECC-aligning
same-stamp frames, SN 33234316), and laptop re-runs of an SVO do not reproduce Jetson behavior
closely enough to stand in for the Jetson `.mcap` files in `data/recordings/`. So the RGB corpus has to be collected fresh, on this camera, at this mount.

Scale is not the worry; input resolution is. At f near 1006 px, a 20 cm robot spans 110 px at the
near mat edge and 52 px at the far corner. For comparison, a 2026-09-07 run over 20 BrettZone
`Cage-N-Overhead-High` fights (178,031 frames of 1080p) put the median `robot` box at 121 px on a
side, which `imgsz 640` presents to the network as 40 px, and lifting to `imgsz 1280` recovered
43% of the missed frames on its own. Downscaled to `imgsz 640` those become 37 px and 17 px, which is worse than the
case that already failed. So `imgsz 1280` is not a preference here, it is the floor.

That is what sizes the compute upgrade. `imgsz 1280` on an Orin Nano at 60 Hz alongside the field
mask model is the thing to benchmark before committing to the box.

## 12. Python tooling

- `auto_battlebot/recording/svo2.py` stays for the SVO corpus. `mcap_io.py` gains a
  `foxglove.CompressedVideo` decoder for `/camera/video`, registered the same way the existing
  protobuf decoders are: `mcap_io.py:91` keys them on `schema.name` rather than the file-local
  schema id, so registration is the whole integration. It is stateful where `imdecode` was not, so
  it holds one codec context per file and yields in stream order; every caller iterates a whole
  recording in order already.
- The tools that read `/camera/image` need no change. Give them an opt-in to read `/camera/video`
  instead, so they can run against a match recording rather than only a label run.
- `export_camera_transforms.py:310` matches dataset images by position in the image stream. On
  `/camera/video` that stays valid only because section 7 sets `max_b_frames = 0`, so decode order
  equals capture order. If B-frames were ever enabled, that matching breaks silently.
- `scripts/combine_mcap_svo.py` is deleted rather than ported. Keep it reachable in git history for
  the SVO corpus.
- `training/svo_export.py` and `training/svo_rgb_extract.py` need video-mcap equivalents before
  any RGB-camera labelling can start.
- `auto_battlebot/recording/mcap_write.py` needs the `FrameMeta` field rename kept byte-identical
  with the C++ schema text.

## 13. Order of work

1. Land the field transform. Section 1a first, because a clipped outline is the expected case at
   this mount and the border guard as specified rejects it outright. Then 1b, the per-cage
   calibration file and `CalibratedFieldFilter`.
2. Upgrade desktop OpenCV to 4.8 (section 2a) and clean-build. Nothing else depends on it, and it
   has to precede the fiducial filter or that code needs version guards it will never shed.
3. Land `FiducialFieldFilter` (section 2). It is independent of step 1, testable against a still
   photo of the board plus a synthetic pose, and it is the mode that unblocks a venue trip before
   any cage has been surveyed. The mirrored-id test comes with it.
4. Make ZED optional. `BUILD_WITH_ZED`, source filtering, guarded registrations. Verify a
   `-DBUILD_WITH_ZED=OFF` build links and that a `-DBUILD_WITH_ZED=ON` build still replays an SVO
   unchanged. This is pure plumbing and can go in parallel with step 1.
5. Camera calibration path. The TOML format, the loader, `initUndistortRectifyMap` at open, and a
   checkerboard capture script. Testable against a still image before any hardware arrives.
6. `V4l2RgbCamera`. Needs the camera and a Jetson.
7. H.264 recording into `McapRecorder` and `VideoPlaybackCamera` reading it back. Rollover, the
   encoder thread, and the three-channel reader. Port the playback tests. At this point the full
   record-and-replay loop works.
8. Turn on `StaticDetectionGate`, replay MassD and AER, and write down what the loss of the height
   gate actually costs.
9. Collect the RGB corpus and retrain.

Steps 1 through 5 and 7 are all testable without the camera in hand. Only 6 blocks on hardware.

## Open questions

- **Mount height and standoff.** These are one question, since only `sqrt(d^2 + h^2)` matters for
  near-corner visibility, and 1.278 m is the threshold at the estimated f. NHRL's own 1.2 m puts us
  just under it at any practical standoff. Going to 1.3 m, or accepting 0.2 m of standoff at 1.26 m,
  clears it. Both are fixture decisions and both need the real calibrated f first.
- **Downtilt.** The window holding the near corners and the far edge in frame at once is only a few
  degrees wide at this vertical field. If the fixture machines a fixed tilt, that angle has to come
  from a measurement on a real cage, not from this arithmetic. A tilt adjustment that locks would
  buy back the margin at the cost of a repeatable pose, which is the thing the quick-connect
  fixture exists to give us.
- **Is the board allowed in the cage between matches?** Section 2 assumes we can lay it down, run
  a ten-second lock, and pull it before the match. If a venue says no, the wall-mounted variant is
  the fallback and needs a place to hang that does not move.
- **Do all NHRL cages use the same mat size?** The companion plan already found nominal 8 ft
  (2.4384 m) runs about 6% long against measured mats of 2.30 to 2.40 m, and per-cage calibration
  makes that a measured constant rather than an assumed one. Worth confirming the cages differ
  before machining a mount per cage.
- **Frame rate.** 1920x1200 at 60 fps costs 276 MB/s over CSI and 6.9 MB per converted frame. If
  the detector cannot keep up at `imgsz 1280`, capture at 1920x1080 at 70 fps or 1280x720 at
  120 fps instead and lose vertical FOV, which is the axis with the least margin at 61.6 degrees.
- **Transmitter link.** Out of scope here, but the non-USB path is a Jetson UART at
  `/dev/ttyTHS1` straight into a CRSF TX module at 400 kbaud. `src/serial/serial_port.cpp` needs a
  device path and baud change and probably a low-latency ioctl. Worth confirming the 40-pin header
  UART is free on whichever carrier board the compute upgrade lands on.

## Next steps

1. Calibrate the lens as soon as it arrives and get the real `fx`. Every geometry number above is
   an estimate off a spec sheet whose three field-of-view figures imply three different focal
   lengths, and one checkerboard run replaces all of it.
2. With the calibrated `fx`, shoot an NHRL cage through the polycarb at 1.2 m and at 1.3 m, and
   confirm how much mat boundary is actually visible at each. That settles the mount height, the
   downtilt, and how much of section 1a is needed.
3. Implement section 1a against `auto_battlebot/perception/field_pose.py` and test it on the
   clipped 2024-10-26 recording that the companion plan already found trips the border guard.
4. Land `BUILD_WITH_ZED` and confirm both build configurations.
5. Benchmark `imgsz 1280` on Orin Nano and Orin NX 16 GB before ordering the compute upgrade.

## What landed

Implemented 2026-09-09 against OpenCV 4.10.0, built from source into `/usr/local` on every
platform by `install/install_opencv.sh` (the generalized `install_opencv_jetson.sh`).

| Step | State |
| --- | --- |
| 1. Field transform, section 1a and 1b | done. `HomographyFieldFilter`, `CalibratedFieldFilter`, the shared `CameraWorldFieldFilter` base, `field_outline.cpp`, `field_pose.cpp`, `cage_calibration.cpp`, 14 unit tests |
| 2. OpenCV 4.10 | done. Installed, `find_package(OpenCV 4.10 REQUIRED ... objdetect calib3d)`, clean-built |
| 3. `FiducialFieldFilter` | done, including the mirrored-id test |
| 4. `BUILD_WITH_ZED` | done. Both configurations build and link |
| 5. Camera calibration path | done. `camera_calibration.cpp`, TOML loader, `initUndistortRectifyMap` at open |
| 6. `V4l2RgbCamera` | written, **never run**. Needs the camera |
| 7. H.264 into `McapRecorder` | done. Encoder thread, rollover, `/camera/video`, `FrameIdentity` change |
| 8. `VideoPlaybackCamera` | done. Nine tests, including the ported SVO cases |
| 9. `StaticDetectionGate` | wired on in `_orin_rgb.toml`, **not measured**. See below |
| 10. RGB corpus and retraining | blocked on the camera |

### Three things the plan got wrong

**The coverage guard is close to dead once the corners are refined.** The companion plan's
"mask over quad area above 1.05" caught the *inscribed* quad, before the edge refinement existed.
With refinement in place the fitted quad circumscribes the mask, so the ratio never rises above 1
on its own: a convex pentagon measures 0.936, an ellipse 0.874, a good mat 0.96 to 0.98. Verified
against `auto_battlebot/perception/field_pose.py`, which gives the same numbers. The guard is
kept because it still catches a refinement that fails and falls back, and it is now measured
against the quad clipped to the image so a clipped field does not read as a bad outline. What it
cannot do is reject a convex outline that is not the mat.

**Three lines determine the pose, but not on their own.** A rectangle mirrored about either of its
own axes is the same rectangle, so a mirrored field frame satisfies all three fitted lines
exactly and the residual cannot tell it from the right answer. It came out 2.35 m wrong, one field
width. The winding of the projected quad is the discriminator, and `pose_from_three_lines` takes
the observed corners for that reason alone.

**`/camera/frame_meta` cannot be paired with `/camera/video` by iteration order.** The two are
written by different threads and carry different clocks: the video message's `log_time` is the
capture instant the encoder carried through, while frame_meta is logged when the publisher reaches
it. Pairing them in stream order ran eleven frames ahead. They are joined on
`video_frame_index` instead, which is exact.

### What is not done

- **Step 9's measurement.** The static gate is on in the RGB configs, but the honest number the
  plan asks for, what the loss of the height gate costs on the MassD and AER recordings, is not
  measured. It needs the logo-versus-moving scoring harness those `_common.toml` comments came
  from, which is not in the tree.
- **Anything needing the camera**: `V4l2RgbCamera` compiles and is wired end to end but has never
  seen a frame, the lens is not calibrated, `config/cameras/ecam25_example.toml` holds spec-sheet
  estimates, and no RGB corpus exists to retrain against.
- **`imgsz 1280` on Orin Nano** is not benchmarked.

### One consequence worth knowing

`FrameIdentity` lost `svo_path` along with the separate-file join it existed for. New ZED
recordings still record which frame of their SVO each tick came from, as `video_frame_index`, but
no longer which file. `scripts/combine_mcap_svo.py` is deleted; `docs/adding_eval_recordings.md`
says how to recover it from git history for the SVO corpus.
