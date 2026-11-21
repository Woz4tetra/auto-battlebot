# AprilTag ground truth tracking, 2026-11-15

Notes on `auto_battlebot/calibration/apriltag/`: the marker detector settings, gamma correction and the capture MCAP storage choices, with the numbers they were tuned against.

## Detection and pose geometry

AprilTag detection and field-plane pose geometry, shared by capture and analysis.

`apriltag_track.py` uses this live (to guide the operator in the preview); `analyze_apriltag_mcap.py`
uses it to re-detect the recorded camera images after the fact. Keeping detection in one module
means the offline poses are solved with the exact detector, gamma correction, and PnP the operator
saw on screen.

Two PnP solves:
  1. solve_floor_extrinsic locks the camera pose relative to a flat AprilTag GridBoard, defining the
     field frame (z = 0 on the floor). Solved once from several frames in which the board is
     visible.
  2. tag_pose_field solves the robot tag's 3D pose each frame and expresses its (x, y, yaw) in that
     frame. Because the pose is solved in 3D, the tag's height above the floor does not bias (x, y)
     the way a floor homography would.

## Detector settings

ArucoDetector tuned for small/distant 36h11 tags (markers ~20-40 px from a 1-1.5 m mount).

The hard limit is pixels-on-target: face-on, 36h11 decodes reliably only above ~18 px edge, and
no parameter recovers a marker below ~15 px (that is what the 1080p default buys). Within that
budget these settings squeeze out the margin and sharpen pose:
  - a tight adaptive-threshold window range (5..21, step 8 -> 3 passes). The big windows (>21)
    were the cost driver at 1080p (33 ms -> ~11 ms per frame, the difference between ~30 and ~60
    fps) and added no detections once auto_gamma normalizes brightness, so they are dropped.
    Verified 15/15 on both a face-on render and an oblique 1080p frame.
  - lower perimeter gate so small markers are not pre-rejected; looser polygon approx tolerates
    the rougher quads low-res markers produce.
  - max error correction: 36h11's Hamming distance (11) leaves plenty of headroom, so accepting
    more corrected bits recovers borderline reads without inviting false positives.
  - subpixel corner refinement: tighter corners -> lower (x, y, yaw) noise, which the plant fit
    needs. It is cheap here (~0-4 ms) so it stays.
minMarkerDistanceRate is left at its default: lowering it interacts with subpixel refinement to
drop a valid marker (it merged duplicate candidates), verified on the board photo (15/15 only at
the default).

## Gamma correction

Brighten an underexposed frame via a gamma curve that lifts its mean toward target_mean.

Arena lighting is often dim and the camera underexposes: a raw frame at mean ~52/255 detects 0
tags because the black/white marker contrast is crushed into the shadows. A power-law stretch
recovers it (0 -> ~10 tags on a real dim frame). Well-lit frames (mean >= min_mean) are returned
untouched, so this never hurts a good exposure. The transform is monotonic and applied per
channel, so marker corner geometry (hence the recovered pose) is unchanged. Applied at detection
time, not when recording, so the MCAP keeps the raw frames and the gamma can be retuned offline.

## Upsampling small markers

`scale` > 1 upsamples the (gamma-corrected) frame before detection, which recovers small markers
sitting near the pixels-on-target limit: the robot tag is only ~40 px at 1080p and its detection
flickers on/off during fast motion. Upsampling 2x lifts the borderline reads over the line
(measured ~+6 pts on the moving frames of a real run). The offline analyzer uses scale=2 for the
robot tag; the live preview leaves it at 1 for speed. Corners are divided back by `scale` before
returning, so PnP solves in the true original-image pixel frame regardless of scale, and the
subpixel refinement done at the higher resolution actually sharpens the corners.

## MCAP chunk compression

JPEG frames are tiny and already compressed, so ZSTD is cheap. Raw frames are 6 MB each and
ZSTD on each one is the capture wall (~26 ms per frame; LZ4 ~13 ms), both under 60 fps. Raw frames
use no compression: ~3.6 ms per frame, lossless. The NVMe carries the ~370 MB/s (~3.6 GB per
~10 s capture).
