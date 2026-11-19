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

## Frames

Written down once here and reused by every fit, render, and grading script:

- **hfield**: the frame `auto_battlebot.perception.field_pose.pose_from_corners` solves in. Origin at the mat centre, z up (r3 . t < 0 for a camera above the mat), and the corner winding of `field_object_corners`: object (-x, -y) binds to the image's top-left corner. For a camera at the near wall looking across the mat that puts hfield +x toward image bottom (the camera) and hfield +y to image right. The `translation` and `rotation` in the TOML are `T_cvcam<-hfield`, with the camera in OpenCV convention (z forward, y down).
- **W** (the Blender world and the true_battlebot map frame): mat centre, z up, +y away from the camera (image top), +x to image right. `WORLD_FROM_HFIELD` is Rz(-90 deg): hfield x -> W -y, hfield y -> W +x.
- **Blender camera**: looks down its own -z with +y up, so `OPENCV_TO_BLENDER_CAMERA` flips y and z, the same thing BlenderProc's `change_source_coordinate_frame_of_transformation_matrix(T, ["X", "-Y", "-Z"])` does.

The chain from a fitted pose to a BlenderProc camera:

    T_W<-cam_cv       = WORLD_FROM_HFIELD @ inv(T_cvcam<-hfield)
    cam2world_blender = T_W<-cam_cv @ OPENCV_TO_BLENDER_CAMERA

The Python writer `save_cage_calibration` emits exactly the keys `src/field_filter/cage_calibration.cpp` parses, so a pose fitted in Python is what `CalibratedFieldFilter` loads at match time. `tests/python/test_cage_calibration.py::test_saved_toml_keys_match_cpp_reader` pins the key set.

## Pose summary

`pose_summary` returns the camera pose in W as a dict. `tilt_from_down_deg` is the angle between the optical axis and straight down. `yaw_deg` is the heading of the optical axis projected on the floor: 0 along +y (looking away from the near wall), positive toward +x. The key stays `yaw_deg` because fitted-pose JSON files and `training/synthetic/synthgen` read it.
