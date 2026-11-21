# 2026-11-15 field init retry

## Field init retry

An init request means "keep trying", not "try exactly this frame". A fiducial board stacks correspondences over several frames before its pose latches, and a mask fit can lose one frame to a robot sitting on the field edge. Bounded, because the retry runs the field-mask model each frame and a request that will never succeed has to stop somewhere.
