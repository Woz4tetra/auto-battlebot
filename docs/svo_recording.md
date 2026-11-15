# SVO recording

`SvoRecorder` manages SVO recording for an open ZED camera. It owns the recording lifecycle: filename generation, enabling and disabling recording on the SDK, evicting old files when the holding directory grows too large, and rolling the active file over once it exceeds the size cap.

## Threading

- `set_desired()`, `start()` and `stop()` are driven from the runner thread; `on_frame_grabbed()` runs on the capture thread. Both call into the same `sl::Camera` handle.
- `lifecycle_mutex_` serializes start, stop and the size rollover, so a toggle cannot land between the rollover's stop and its start. It is held across the SDK calls in start, stop and rollover, and never taken per frame.
- The state mutex guards `current_path_` and `frame_index_`. It is held briefly and never across an SDK call.
- In the Runner, `recording_mutex_` is held by `apply_recording` and by camera reinitialization, which both start and stop the camera's encoder or SVO recorder.
