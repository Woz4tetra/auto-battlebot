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

## Workarounds

- **Frame index before rollover.** `on_frame_grabbed()` claims the frame's index in the file that is open at that moment, before the size check can roll the file over, so the return value names the file the frame went into. Stop or a recording turned off since the size check skips the rollover, which would restart a recording that is already cut.
- **Open that cannot be cancelled.** `ZedDevice::open` runs `zed_.open()` on a background thread and polls `cancel` while the SDK call blocks. The SDK has no cancel, so a cancelled open waits for it to return before leaving: the handle must never be used concurrently or destroyed mid-open. After `kOpenHardTimeout` the future is moved into `g_leaked_open_futures`, a static that is never freed, and `OpenResult::Leaked` tells the caller not to touch the handle. A `std::async` future blocks in its destructor on a task wedged inside the SDK, so destroying it would hang shutdown. `ZedRgbdCamera`, `ZedOneRgbCamera` and `ZedSvoPlaybackCamera` skip `close()` on a leaked open for the same reason. The SDK wedge itself is not reproduced here.
- **Capture thread stuck in `grab()`.** `ZedRgbdCamera` detaches a capture thread that does not join in time. The destructor then skips `svo_recorder_->stop()` and `zed_.close()`, since they would race the SDK on the same handle, and lets the OS reap on exit. `initialize()` sets `should_close_` and returns false, so the runner exits and systemd restarts it under `Restart=on-failure`; reopening the handle under a thread still in `grab()` would race. `ZedOneRgbCamera` waits `kCaptureThreadJoinTimeout` (3 s) and calls `std::_Exit`: joining hangs until systemd kills the process 90 s later, and detaching leaves the thread in `grab()` on a freed camera. `Runner::run()` closes the recordings before any destructor runs. Unverified: the `grab()` hang is not reproduced.
- **`get()` timeout.** `get()` gives up with no frame past its timeout so the runner gets its tick back to read quit and reboot commands and to reinitialize the camera. A healthy ZED never goes this long. Every captured frame carries depth, so `get()` takes whatever the capture thread last published and never requests one and waits.
- **Frame identity.** `TIME_REFERENCE::IMAGE` runs about half a frame ahead of what the SVO recorder writes for the same grab, so a timestamp alone cannot join recorded output to SVO frames. Each frame also records which camera frame it is. `TIME_REFERENCE::IMAGE` is the capture instant live and the original recording time in replay, which lets a `ManualClock` drive the pipeline off the frame stamp.
- **Fresh `cv::Mat` per frame.** `cv::Mat::create()` reuses an allocation whenever size and type match, without consulting the reference count. Consumers hold a shallow `cv::Mat` sharing the buffer, so converting into the same destination would overwrite pixels a consumer still reads one frame behind. Each frame converts into a new Mat. Both Mats take the SDK's own step: the default `AUTO_STEP` assumes `width * elem_size`, which the SDK does not promise, and on a padded allocation it reads each row further into the previous one and shears the image down the frame.
- **Replay grab errors.** A file does not disconnect, so a grab error other than end of file is not recoverable by retrying and ends replay.
