# Pitch noise on a flat spin, 2026-10-14

The recording `2026-10-14_23-11-27` is a flat spin. Tag pitch frame noise is 1.7 deg, with lone single-frame spikes up to 7.8 deg.

- A pitch gate that trips on one frame marks flat windows as nose lifts.
- `_pitched` ignores single-frame spikes, and a sustained 15-frame (quarter second) lift at 6 deg trips it.
- `test_pitch_gate_ignores_single_frame_spikes` reproduces the noise with spikes of -5.1, 7.8, 4.8 and -5.8 deg on 1.5 deg Gaussian noise.
