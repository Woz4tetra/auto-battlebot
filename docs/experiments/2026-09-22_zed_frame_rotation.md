# ZED frame rotation chase

`ZedDevice::retrieve()` logs where an SDK buffer sat last time, and reports when one moves. It was added while chasing frames that came out cyclically rotated left by 235 px, which is 940 bytes at 4 bytes per pixel. A constant per-row shift like that is a moved base pointer, not a stride mismatch, and the SDK reuses these Mats, so the geometry should never change after the first frame. The log is silent in steady state: one line at startup, one only if something moves.

The root cause turned out to be uvcvideo latency, fixed 2026-09-22 (SCHED_FIFO uvcvideo kworkers via `install_uvcvideo_rt.sh`), so the buffer-move logging may be removable.
