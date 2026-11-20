# Camera calibration files

Where each file in `config/cameras/` came from, and what it can and cannot be trusted for. The
file format and loader are in `docs/reference/perception/cage_calibration.md` (Camera intrinsics).

## `zed2i_720p.toml`

The deployed ZED 2i at HD720, read off the robot's own recordings.

- Source: `/camera/camera_info` in 17 saved MCAPs from `MassD_2026-08-29` and `NHRL_2026-05-02`.
  Every one reports 1280x720 with the same principal point and zero distortion, because the ZED
  SDK hands over an already-rectified image.
- `fx` spans 526.858 to 528.247 across recordings (0.26%, the SDK's temperature-dependent focal
  readout). The file holds the mean.
- 101.0 degrees horizontal, wider than the 95.2 degree NHRL phone in `brettzone_cage_high.toml`.
  Normalized focal length differs by 10%, so the two are not interchangeable: synthetic cage frames
  that stand in for our camera render through this file, and only footage shot on NHRL's phone
  renders through that one.
- HD720 is not a scaled HD1080. The one 1080p recording in the corpus reports `fx = 1064.4`, which
  is fx/width 0.554 against 0.412 here: HD1080 crops the sensor where HD720 bins the full width. Do
  not rescale this file to 1080p.
- Distortion is zero by construction, so `rectify_maps` in `auto_battlebot.perception` is a no-op at
  any alpha. The cage renderer depends on that.

## `brettzone_cage_high.toml`

NHRL's fixed `Cage-N-Overhead-High` feed, as re-wrapped by `scripts/video_to_mcap.py`.

- Measured with `cv2.calibrateCamera` over a fiducial board in the `true_battlebot` repo
  (`ros_ws/bw_camera_calibration/.../intrinsic_calibration.py`), stored there as
  `perception/configs/metrics_tool/nhrl_camera_intrinsics.toml` at 1280x720. That calibration is a
  3x downscale of the 3840x2160 source feed, so the file holds the 1280x720 result scaled by 1.5 to
  1080p. Distortion coefficients are normalized and carry over unscaled.
- `fx = 876.5` at 1920 px is a 95.2 degree horizontal field, about a 16 mm equivalent. A 24 mm
  equivalent guess (`fx = 1331`) was wrong by half: the overhead frame shows the crowd on both sides
  of an 8 ft cage from 1.7 m up, which no 72 degree lens covers.
- The calibration comes from 2024-10-26 footage. If NHRL has swapped the phone since, these drift.
  Re-run the board calibration, or fall back to `FiducialFieldFilter`, when a field pose looks off.

## `synthetic_cage_lens_118deg.toml`

Estimated, not measured. The lens the synthetic cage renders use: a fisheye-class M12 lens,
approximated by the 5-coefficient plumb_bob model the C++ `Rectifier` uses and fitted to three
published fields of view: 104.6 degrees horizontal, 61.6 vertical, 128.2 diagonal.

Derivation, on the sensor's native 1920x1200 mode (AR0234CS, 3 um pixels, 5.76 x 3.6 mm active
area, half-extents 960 / 600 / 1132.08 px):

1. The three quoted fields fit an odd fisheye polynomial `f(t + a3 t^3 + a5 t^5)` exactly, at
   `f = 1156.2 px`, `a3 = -0.1259`, `a5 = +0.0209`. That curve is the best available description of
   the lens.
2. `fx`, `k1` and `k2` pass through all three quoted fields (residual under 0.06 px) while `r_d(r)`
   stays monotonic out to the 64.1 degree image corner.

A k1/k2/k3 least-squares fit tracks the fisheye curve three times better (18.8 px worst error
against 54.5 px) and is unusable: `r_d(r)` turns over at 63.3 degrees, inside the corner, so
`initUndistortRectifyMap` folds the corners a field fit needs. Do not swap in the lower residual.

The file is written for the 1920x1080 mode (70 fps). Same `f` and distortion as the 1920x1200 fit,
with `cy` recentered, because 16:9 is a crop of the sensor. `CameraCalibration.scaled` scales `fx`
and `fy` independently, so handing a 16:10 calibration to a 16:9 render would stretch the image.

Rectified at alpha 1.0 this gives fx 576.4, a 118.0 degree horizontal field, and a 36.2% black
border.

## Replacing an estimate

Shoot a checkerboard through an offcut of the cage panel at the mounted standoff. A flat sheet in
the optical path refracts more toward the frame edges, which is where the field boundary sits, and
calibrating in free air folds that error into the field pose where no reprojection residual shows
it. If plumb_bob will not converge below about half a pixel, `cv::fisheye` is the correct model and
`camera_calibration.cpp` needs a second code path.

## Python rectification mirror

`auto_battlebot.perception.camera_calibration` mirrors `include/rgbd_camera/camera_calibration.hpp`: the same TOML keys, and `rectify_maps` reproduces `Rectifier::build` (`getOptimalNewCameraMatrix` with alpha 1.0, then `initUndistortRectifyMap`), so a frame rectified in Python matches what playback hands the perception stack. Anything that compares against those frames, a fitted camera pose or a rendered image, must use the rectified matrix `rectify_maps` returns, not the calibrated one. Alpha 1.0 keeps every source pixel, which is what the C++ side chooses so the mat edges are never cropped away. `tests/python/test_ecam25_calibration.py::test_rectify_maps_default_keeps_every_source_pixel` pins the alpha.

## C++ rectifier

`Rectifier::build` takes `alpha` 1.0 by default. A cropping alpha at this field of view cuts exactly the mat edges the field fit needs. `RectifierTest.KeepsEverySourcePixelByDefault` pins it.

`size` may differ from the calibration's. The intrinsics scale to it and the distortion coefficients do not, so a calibration shot at 1920x1200 still rectifies a 1280x720 capture (`RectifierTest.ScalesIntrinsicsToCaptureSize`).

`camera_info()` returns the rectified matrix with D set to five zeros, so `camera_info` keeps emitting a plumb_bob model that happens to be trivial. `CameraCalibration::distortion_k1_k2_p1_p2_k3` holds the plumb-bob coefficients in that order.
