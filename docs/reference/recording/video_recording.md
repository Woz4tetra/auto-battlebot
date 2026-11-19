# Video recording

Encoder settings for the H.264 recording written into the MCAP `/camera/video` channel.

- Bitrate: 15 Mbps is 1.9 MB/s, about 6.8 GB per hour, and a three-minute match is about 340 MB.
- Keyframes: an IDR every half second at 60 fps. This sets the seek granularity on playback and bounds how far a decoder has to run forward from a chunk boundary.
- Input: UYVY straight off the V4L2 buffer, rather than the BGR copy the detectors use. libswscale goes UYVY to NV12 without the detour, and it means the recording is pre-rectification, so a revised calibration can be applied to footage already shot. The ZED X One hands over BGRA, which the Jetson's nvvidconv takes directly; it has no 3-channel BGR input.
- Preview: the 640-wide dashboard preview is not a recording. It may fall back to libx264 on aarch64, at the ultrafast preset on one thread: the Orin Nano has no NVENC, and there libx264 costs about 9% of one core at 640x360 30 fps.

## Encoder backends

`VideoEncoder` owns the queue, the encode thread, and the pts to capture time map. Each `H264Backend` (FFmpeg or GStreamer) only turns frames into Annex-B access units. Every backend must put SPS/PPS in band ahead of every IDR and emit no B-frames, so a decoder can start at any IDR and the nth packet is the nth captured frame. `encode()` takes the frame by value so a backend can hand its memory to the encoder without a copy. `finish()` flushes: every frame `encode()` accepted reaches the sink before it returns. `tests/camera/test_h264_backend.cpp` pins these per backend.

## Playback seek

`VideoDecoder` is stateful where `cv::imdecode` was not: it holds a codec context for the life of a file and has to be fed in stream order. Seeking means finding the last IDR at or before the target and decoding forward from there, never jumping into the middle of a GOP. `sl::Camera::setSVOPosition` has the same problem, landing on non-keyframes so the next grabs return smeared RGB, and it applies to any inter-coded stream. `VideoPlaybackCameraTest.SeekStartsFromPrecedingKeyframe` compares a seeked frame against the same frame decoded from the start.
