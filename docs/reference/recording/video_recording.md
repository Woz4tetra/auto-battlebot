# Video recording

Encoder settings for the H.264 recording written into the MCAP `/camera/video` channel.

- Bitrate: 15 Mbps is 1.9 MB/s, about 6.8 GB per hour, and a three-minute match is about 340 MB.
- Keyframes: an IDR every half second at 60 fps. This sets the seek granularity on playback and bounds how far a decoder has to run forward from a chunk boundary.
- Input: the ZED X One hands over BGRA, which the Jetson's nvvidconv takes directly; it has no 3-channel BGR input.
- Preview: the 640-wide dashboard preview is not a recording. It may fall back to libx264 on aarch64, at the ultrafast preset on one thread: the Orin Nano has no NVENC, and there libx264 costs about 9% of one core at 640x360 30 fps.

## Encoder backends

`VideoEncoder` owns the queue, the encode thread, and the pts to capture time map. Each `H264Backend` (FFmpeg or GStreamer) only turns frames into Annex-B access units. Every backend must put SPS/PPS in band ahead of every IDR and emit no B-frames, so a decoder can start at any IDR and the nth packet is the nth captured frame. `encode()` takes the frame by value so a backend can hand its memory to the encoder without a copy. `finish()` flushes: every frame `encode()` accepted reaches the sink before it returns.

The GStreamer backend lets its `appsrc` hold 2 frames. Past that, `encode()` waits up to 1000 ms for the pipeline, then drops the frame. `VideoEncoder`'s own queue absorbs bursts, and a push into a stalled pipeline must not hang `stop()`. The pipeline carries 1920x1200 at 60 fps on the ZED Box, so this is a stall guard.

`VideoEncoderOptions::queue_depth` (4) is how many frames the encoder may fall behind. At that depth `submit()` drops the frame and counts it in `dropped_frames()` instead of waiting on the encoder, because a wait from the perception loop would be a blocking call inside the latency budget. A counted drop is visible; a silent one is not.

`submit()` returns the frame's index in the encode session, which is the position of its packet in `/camera/video` for that recording, or -1 when the frame was not taken (stopped, queue full, or broken). Cameras write that index into `frame_meta`. Only the encoder knows which frames became packets, so a counter kept anywhere else drifts at the first drop.

When a backend refuses a frame that already has an index, the encoder latches `broken()`. `submit()` takes nothing from then on, since every index handed out afterward would point past the last real packet. Only `stop()` then `start()` clears it, which is the caller restarting the recording. `stop()` calls `finish()` on the backend, so frames still queued reach the sink.

Candidate order is FFmpeg `h264_nvenc` (desktop), FFmpeg `h264_nvv4l2m2m`, GStreamer `nvv4l2h264enc` (Jetson hardware encoder), then libx264 (FFmpeg, else GStreamer `x264enc`). A candidate counts only if it opens: the ZED Box's stock FFmpeg registers `h264_nvenc` but cannot open it without `libnvidia-encode`. On aarch64 libx264 is allowed only for `software_preview`; the recording itself needs a hardware encoder (measurement in `docs/experiments/perception/2026-11-19_libx264_orin_encode_rate.md`).

## Playback

`VideoPlaybackCamera` joins `image_stamp_ns` from `/camera/frame_meta` to a video packet on the frame ordinal (`video_frame_index`), never on the timestamp: the two channels are logged by different threads on different clocks (see `foxglove_recording_format.md`).

The camera indexes the video channel once at `initialize()` so a `start_frame` can find its IDR. The `/camera/frame_meta` stamps are indexed in the same pass, keyed on the frame ordinal that both channels agree about.

Real-time pacing sleeps until a deadline, not for one interval per frame. The first frame anchors a wall-clock instant to its stamp, and every later frame is due at the anchor plus its offset in the recording's timeline. A lag over 500 ms re-anchors.

## Playback seek

`VideoDecoder` is stateful: it holds a codec context for the life of a file and has to be fed in stream order. Seeking means finding the last IDR at or before the target and decoding forward from there, never jumping into the middle of a GOP. `sl::Camera::setSVOPosition` has the same problem, landing on non-keyframes so the next grabs return smeared RGB, and it applies to any inter-coded stream.

## Keyframe requests

The request fires from a probe on the encoder's input pad as the requested frame arrives, so it lands on that frame however many frames `appsrc` held ahead of it. `nvv4l2h264enc` takes a force-IDR action and `x264enc` takes the force-key-unit event. `nvv4l2h264enc` applies force-IDR to the next frame the hardware encodes, so a hardware encoder a frame behind puts the IDR one frame before the requested one. That still lets a new preview viewer decode at once. FFmpeg forced keyframes are IDR, not just I-frames, and encoders without the option ignore it.

## Encoder failure

A session recorded without its video cannot be replayed, and nobody finds out until the morning after. `ZedOneRgbCamera` fails `initialize()` if no H.264 encoder opens. Turning `video_recording` off in the config is the explicit way past it.

## Foxglove stamps

Foxglove's decoder rejects a frame whose timestamp is not after the previous one, so `log_time_ns` on each `CompressedVideo` is strictly increasing. Foxglove image panels reject a calibration whose frame differs from the image's, so `camera_info.header` is copied from the image header.
