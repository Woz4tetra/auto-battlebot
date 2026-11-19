# Cage calibration

`CageCalibration` is a cage measured once, so match day does not have to fit anything. `playground/field_transform/calibrate_cage.py` writes it from a clean empty-cage frame and it is loaded unchanged at startup. Everything in it is a measurement of one physical cage with the camera fixture seated, so it travels with the venue rather than with the build. Seating check and match-day behavior: `docs/reference/perception/robot_filter_pipeline.md`.

## Camera intrinsics

The ZED handed us rectified frames and near-zero distortion coefficients. A 104.6 degree M12 lens does not, and every downstream consumer assumes a pinhole: the homography fit, keypoint projection, the UI marker overlay, and the trained detectors. So the camera rectifies before publishing, and the `CameraInfo` it emits carries the rectified matrix with D zeroed.

Calibrate through polycarbonate. The camera sits behind a cage panel, and a flat sheet in the optical path refracts increasingly toward the frame edges, which is exactly where the field boundary sits. Calibrating in free air folds that error into the field pose where no reprojection residual will ever show it.

## Calibration file

`save_cage_calibration` writes `calibration_id`, `field_size_x` / `field_size_y`, `translation`
(the field centre in camera coordinates) and `rotation` (row-major 3x3). The tool writes it from a
clean empty-cage frame with the fixture seated; `config/cages/example_cage.toml` is a placeholder
in that shape. `field_size` is the floor mat measured at that venue, not the nominal cage: 8 ft
(2.4384 m) runs about 6% long against measured mats of 2.30 to 2.40 m, and the size enters the pose
linearly.

