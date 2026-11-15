# Sysid YOLO dropout fill, 2026-10-15

The sysid tag smoother fills AprilTag dropouts with YOLO robot detections projected onto the field. The numbers below were measured on the 2026-10-15 sysid sessions.

## Implementation notes

### `perception/tag_pose_smoother.py`

#### `_fuse_yolo`

Position only. Across 40 withheld 1 s tag spans the keypoint heading was worse (4.1 deg
median) than the heading the IMU yaw rate carries through the gap (1.8 deg), so it is out.

### `perception/yolo_field_fixes.py`

YOLO robot detections as field-frame position fixes for the sysid tag smoother.

The overhead tag drops out while the robot sits flat and in full view: the camera looks down at
an angle, so with the robot heading away from it the 11 degree nose-down tag foreshortens to about
24 px and stops decoding. The deployed YOLO pose model still sees the robot in those frames. Its
front and back keypoints, projected onto a horizontal plane, give the axle centre to about 8 mm,
three times worse than the tag but far better than extrapolating through a second-long gap. Across
40 withheld 1 s tag spans the smoothed position error fell from 95 mm to 8 mm (median). The
keypoint heading (about 5 degrees) is worse than what the IMU yaw rate carries through a gap, so
the smoother takes position only.

Everything the projection needs is fitted per session against the tag, on frames where both see
the robot:

- ``offset_ns``: the video stamp runs ahead of the tag's image stamp by a frame and a half. The
  robot's motion picks the frame; the stamps of frames both streams carry then pin it to the
  stamp-level offset, so a video frame lands within about a millisecond of its camera frame.
- ``plane_height_m``: the height of the plane the keypoints are projected onto. It absorbs where
  the labelled keypoints sit on the chassis, so it need not be a physical height.
- ``axle_offset_m``: from the keypoint midpoint to the axle centre, along the heading.
- the noise, along and across the camera's view, and of the heading.
- size gates. The model sometimes labels the floor marker board as the robot (a 380 px box
  against the robot's 110 to 200 px, keypoints 0.24 to 0.30 m apart against 0.11 to 0.19 m), so a
  detection outside the range the co-visible inliers span, widened by ``GATE_MARGIN``, is dropped.

Fixes fill tag dropouts only: video frames between two tag detections more than
``TAG_DROPOUT_MIN_GAP_NS`` apart. Between closer tag fixes YOLO adds little and is three times
noisier, and
a fit horizon would score against it instead of a tag a few milliseconds away. They are also used
only while the robot is upright and flat by its own IMU: a wheelie or a flip moves the keypoints
off the plane they are projected onto.

#### `select_fixes`

The app drops to about 12 fps once the tag is lost, because the full-frame search is slow, so
most video frames in a dropout were never processed at all. Not every one gets a fix.
