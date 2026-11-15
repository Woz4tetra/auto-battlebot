# e-CAM25 camera over raw V4L2

`V4l2RgbCamera` drives the e-CAM25_CUONX over raw V4L2, rectified in the camera, recorded as H.264.

- Raw ioctls rather than `cv::VideoCapture` or GStreamer. `cv::VideoCapture` hides the buffer count and does not expose `v4l2_buffer.timestamp`, which is the kernel's monotonic capture instant and the thing `sl::TIME_REFERENCE::IMAGE` used to give us. The ManualClock replay path depends on the frame stamp being that instant. GStreamer with nvv4l2camerasrc adds a dependency and a queue of unknown depth, and queue depth is latency in a 60 ms budget.
- `CameraData::depth` is left empty. `KeypointHeightGate` abstains, `to_field_point_cloud` returns nullopt and the publisher already handles it, and the field filter is one of the RGB ones.
- `tf_visodom_from_camera` is identity, because a clamped camera does not move.
