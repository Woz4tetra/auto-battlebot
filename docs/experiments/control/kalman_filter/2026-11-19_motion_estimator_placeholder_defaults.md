# Motion estimator placeholder defaults

Provenance of the `KalmanMotionEstimator` defaults that were never fitted. Field meanings are in `docs/reference/config/fields.md`.

- `opponent_accel_psd = 30.0`: placeholder until measured from fight replays. Sized from about 10 m/s^2 peak accelerations sustained over about 0.3 s maneuvers.
- `keypoint_position_sigma_m = 0.03`: placeholder until the joint jig and camera session measures R.
- `keypoint_heading_sigma_rad = 0.15`: placeholder until the joint jig and camera session measures R.

## Next steps

- Replace each placeholder with a fitted value when the replay measurement and the joint jig and camera session run.
