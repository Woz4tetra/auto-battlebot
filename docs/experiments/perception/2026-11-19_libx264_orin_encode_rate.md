# libx264 encode rate on the Orin

Measurement behind `VideoEncoder` allowing libx264 on aarch64 only for `software_preview`.

- Setup: libx264 through FFmpeg at 1920x1200 on a Jetson Orin using all eight CPU cores.
- Result: 39 to 43 fps.
- Consequence: that is below the 60 fps camera rate and takes every core, which would starve perception. The recording path therefore requires a hardware encoder on aarch64, and libx264 serves only the 640-wide dashboard preview (`docs/reference/recording/video_recording.md`).

The date is the day this was moved out of a header comment; the measurement itself predates it and the source comment recorded no date or box.
