# Forward-speed observation noise (2026-08-23 20-37 recording)

Provenance for the our-robot forward-speed observation in `KalmanMotionEstimator`.

- Position noise: 2.7 mm median residual about a 9-frame smooth on the 2026-08-23 20-37 recording. The config uses 3.5 mm to leave room for the smoother following some of the noise. It is deliberately not `keypoint_position_sigma_m`, which is a conservative placeholder that also sets the position gate; differencing needs the real frame-to-frame scatter.
- Sigma floor: 0.168 m/s median absolute residual against a 150 ms chord reference on the same recording, which is 0.25 sigma if Gaussian. Position scatter is not the whole error: the chord understates an arc, the projection inherits the heading error, and the two endpoint stamps are not exact.
- Baseline: one frame is too short. At 30 Hz and a 0.03 m position sigma the difference carries about 1.3 m/s of noise, so the reference pose is held until the gap reaches the configured minimum (about three frames) and the sigma falls with it. Nothing else in the update observes v directly; without this the speed the motion profile brakes on is whatever the plant model predicted.
