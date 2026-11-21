# SVO recording

`SvoRecorder` manages SVO recording for an open ZED camera. It owns the recording lifecycle: filename generation, enabling and disabling recording on the SDK, evicting old files when the holding directory grows too large, and rolling the active file over once it exceeds the size cap.

## Threading

- `set_desired()`, `start()` and `stop()` are driven from the runner thread; `on_frame_grabbed()` runs on the capture thread. Both call into the same `sl::Camera` handle.
- `lifecycle_mutex_` serializes start, stop and the size rollover, so a toggle cannot land between the rollover's stop and its start. It is held across the SDK calls in start, stop and rollover, and never taken per frame.
- The state mutex guards `current_path_` and `frame_index_`. It is held briefly and never across an SDK call.
- In the Runner, `recording_mutex_` is held by `apply_recording` and by camera reinitialization, which both start and stop the camera's encoder or SVO recorder.

## Playback timing

`ZedSvoPlaybackCamera` warns at startup when `rebase_stamps = false` or `svo_real_time_mode = false`. Both change what the replay measures and neither shows up as an error, so the warning keeps a replay run from being read as if it reproduced field timing.

Without rebasing, frame stamps stay on the recording's clock while the control loop reads the wall clock. Everything that compares the two degrades quietly. The filter's latency compensation holds every prediction instead of leading, and the pipeline latency diagnostic reports the age of the recording rather than the age of the frame.

## Frame index

`on_frame_grabbed()` claims the frame's index in the file that is open at that moment, before the size check can roll the file over, so the return value names the file the frame went into. Stop or a recording turned off since the size check skips the rollover.

## Frame retrieval

`get()` gives up with no frame past its timeout so the runner gets its tick back to read quit and reboot commands and to reinitialize the camera. Every captured frame carries depth, so `get()` takes whatever the capture thread last published and never requests one and waits.

## Frame identity

`TIME_REFERENCE::IMAGE` runs about half a frame ahead of what the SVO recorder writes for the same grab, so a timestamp alone cannot join recorded output to SVO frames. Each frame also records which camera frame it is. `TIME_REFERENCE::IMAGE` is the capture instant live and the original recording time in replay, which lets a `ManualClock` drive the pipeline off the frame stamp.
