# Camera-following clock

`CameraFollowingClock` is logical time anchored to the camera and advanced by the wall clock between frames. `now()` is the newest frame stamp plus the real time elapsed since the Runner handed it over, so `now() - frame_stamp` is this machine's actual processing age of that frame, whatever timeline the stamps live on. That is the quantity the filter's forward propagation needs, and it is the only one of the three clocks that reports it correctly off a recording.

- `SystemClock` gets it right only when camera stamps are wall-clock, which SVO playback stamps are not: unrebased they sit on the recording's clock (a gap of days), and rebased they carry a fixed offset captured during the startup transient that measured about 790 ms of the wrong sign on a 70 s replay. Either way every propagation short-circuits and the emitted estimate freezes at the shutter.
- `ManualClock` reports zero elapsed time, so it never coasts between frames at all.
