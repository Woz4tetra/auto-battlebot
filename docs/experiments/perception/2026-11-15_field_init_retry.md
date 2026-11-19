# 2026-11-15 field init retry

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/runner.cpp`

Beside `field_init_attempts_remaining_ = kFieldInitAttempts;`:

An init request means "keep trying", not "try exactly this frame". A fiducial board stacks correspondences over several frames before its pose latches, and a mask fit can lose one frame to a robot sitting on the field edge. Bounded, because the retry runs the field-mask model each frame and a request that will never succeed has to stop somewhere.
