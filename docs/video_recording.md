# Video recording

Encoder settings for the H.264 recording written into the MCAP `/camera/video` channel.

- Bitrate: 15 Mbps is 1.9 MB/s, about 6.8 GB per hour, and a three-minute match is about 340 MB.
- Keyframes: an IDR every half second at 60 fps. This sets the seek granularity on playback and bounds how far a decoder has to run forward from a chunk boundary.
- Input: UYVY straight off the V4L2 buffer, rather than the BGR copy the detectors use. libswscale goes UYVY to NV12 without the detour, and it means the recording is pre-rectification, so a revised calibration can be applied to footage already shot. The ZED X One hands over BGRA, which the Jetson's nvvidconv takes directly; it has no 3-channel BGR input.
- Preview: the 640-wide dashboard preview is not a recording. It may fall back to libx264 on aarch64, at the ultrafast preset on one thread: the Orin Nano has no NVENC, and there libx264 costs about 9% of one core at 640x360 30 fps.
