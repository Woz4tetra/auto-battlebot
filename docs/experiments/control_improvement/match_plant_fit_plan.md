# Match plant fit plan: Mrs Buff Mk3 from the NHRL May 2026 recordings

Fit the `auto_battlebot/plant.py` model ladder for Mrs Buff Mk3 from match driving instead of
jig protocols, seeded from the jig Stage A values, and verify it by open-loop prediction on
held-out fight recordings. The jig fit failed the EKF acceptance criteria mainly because the
coupling grid produced zero passing runs and a single garage session cannot measure
session-to-session spread. Match driving is nothing but combined v+w excitation on the floor
that matters, so it attacks both failures directly.

Consumers are the same two the jig fit was built for: `stage4_plant_backed_nav_plan.md`
(MotionProfileNavigation constants) and the EKF plant slot in
[`../kalman_filter/kalman_filter_plan.md`](../kalman_filter/kalman_filter_plan.md), which stays
empty until a fit passes acceptance.

## Recording assignments

All from `data/saved_recordings/NHRL_2026-05-02/`. Our robot is Mrs Buff Mk3 in every match;
replay overlays extend `playback/mrs_buff_mk3_playback` (not the /replay skill's mr_stabs
default).

| Recording | Role | Reason |
|---|---|---|
| `2026-05-01_17-42-20` | fit | long uninterrupted driving |
| `2026-05-02_11-45-05_repaired` | validation | clean fight (sphinx) |
| `2026-05-02_14-12-25_repaired` | validation | clean fight (wreckcreation), replays fully on desktop |
| `2026-05-02_15-35-00_repaired` | excluded | robot upside down most of the match; detector does not recognize the inverted robot yet |
| `2026-05-02_16-18-05_repaired` | excluded | embedded in the opponent; only a few seconds of useful autonomy at the start |
| `2026-05-02_17-26-12_repaired` | excluded | heavy damage, large manual-driving chunks |
| `2026-05-02_10-06-02_repaired` | excluded | not assigned; desktop replay at the prescribed start frame 8000 hits a corrupted SVO frame and segfaults |
| massD recordings | excluded | not useful |

Known dropout caveat from the driver: some stretches have the robot not responding to
commands. These must not enter the fit (see the window gate below), and the flagged list is a
deliverable of its own.

## Step 1: regenerate pose tracks through playback

The detector improved since May, so the recorded tracks are stale. Re-run each SVO through
desktop playback per the /replay skill flow: scratch overlay extending
`playback/mrs_buff_mk3_playback`, prescribed `svo_start_frame` from `config/playback/_playback.toml`
(17-42-24 at 0, 11-45-08 at 13000, 14-12-27 at 1540), `svo_real_time_mode = false`, record the
replay mcap. The regenerated `/tf` and `/robot_markers` are the pose source.

Two things in the replay mcaps must be ignored:

- The replay `/diagnostics` command stream comes from PlaybackTransmitter and the replay nav,
  not the May driver. It is not the plant input.
- Replay stamps are rebased onto the current wall clock, so nothing in a replay mcap joins to
  the originals by timestamp.

## Step 2: join original commands to regenerated poses

The May recordings predate `/camera/frame_meta` (added 2026-08-04, 13751b8), so the exact
`svo_frame_index` key only exists on the replay side. The join goes through the SVO file, whose
frame stamps are on the original Jetson clock:

1. Replay mcap: pose at tick, keyed by `/camera/frame_meta` `svo_frame_index`.
2. SVO file (`auto_battlebot/svo2.py`): `svo_frame_index` to original-clock stamp.
3. Original mcap `/diagnostics`: `ch_linear` / `ch_angular` / `ch15` from the
   `opentx_transmitter` `channels` entries, already on the original clock.

SVO image stamps sit roughly half a frame before pipeline stamps and the exporter drops
occasional frames, so leg 2 to 3 aligns on frame interval with a drop tolerance, never
nearest-timestamp. The residual constant offset (about half of 33 ms) is absorbed by the
fitted `delay_s` and noted in its provenance.

Commands are the transmitted values, post trainer-mode mix of navigation and driver sticks.
That is the true plant input whether the nav or the driver was steering, but only while the
auto switch was up; in manual segments the logged channels did not drive the robot.

## Step 3: window extraction

Cut each recording into open-loop scoring windows:

- 0.5 to 2.0 s, `ch15` auto-switch up for the whole window.
- Continuous our-robot track: no perception gap longer than one frame interval. Perception
  dropouts split windows rather than being interpolated over.
- No contact: drop windows overlapping wall proximity or opponent proximity below a threshold,
  or a finite-difference acceleration spike beyond what `k_fwd`/`tau` can produce. Contact
  windows go to a separate list for a later impact model, not into this fit.
- Tag maneuver class per window: straight, arc, spin, reversal, stop.

Radio-dropout gate: after a first robust fit pass, flag windows where the commanded magnitude
is large but predicted-minus-observed displacement says the robot did not respond. Flagged
windows are excluded from the final fit and written to `nonresponsive_windows.csv` with
original-clock timestamps. That file doubles as the evidence base for a future live "robot is
not responding" alert, which the driver asked for.

Target counts: a few hundred fit windows from 17-42-20, held-out split by contiguous time
blocks (first 70% fit, last 30% holdout), not randomly, so the holdout is not interleaved with
training data.

## Step 4: the fit

- Model: the existing ladder in `plant.py` (M0 static map through M4/M5), one code path,
  scored with `predict_windows`. Climb the ladder and keep the highest rung that improves
  holdout, not train, error.
- Seed: `playground/calibration/out/plant_stageA.toml` (jig, model M4). Bounds: jig error bars
  widened 3x for gains and taus (different floor, different battery), deadzones held at the
  jig bracket midpoints unless the data moves them (fights rarely dwell at the threshold, so
  match data cannot resolve them; the jig brackets are the best available and stay as priors).
  Coupling terms `c_sb`, `c_ad`, `c_drift`, `c_drift_bias` get wide bounds; this is the data
  the jig grid never produced.
- Objective: mean windowed open-loop error at 100/250/400 ms horizons via `predict_windows`,
  position plus heading with the weighting the EKF criteria use.
- Optimizer: `scipy.optimize.least_squares` from the seed; CMA-ES fallback if the coupling
  terms stall in a local minimum. Bootstrap over windows for parameter error bars, written to
  `[plant.provenance]` in the output TOML in the existing format.
- Code: a match-window loader next to the jig loader in `auto_battlebot/calibration/`,
  registered as a new catalog entry in `fit_jig_plant.py` (the docstring already prescribes
  this for new excitation sources). Decoding reuses `auto_battlebot/mcap_io.py` and
  `auto_battlebot/diag_io.py`. The earlier per-axis AR fit in
  `playground/control_stage0/fit_plant.py` stays as prior art; it fits a simpler model class
  and does not use the ladder or `predict_windows`.

## Step 5: verification

Three plants scored on identical windows, same scorer, no code branches:

- A: jig Stage A seed, unfit.
- B: this match fit.
- C: current sim values (`simulation/sim_mrs_buff_mk3.toml`), as the regression reference.

Gates, in order:

1. **Holdout (within 17-42-20).** B beats A at every horizon on the held-out 30%. Report
   against the EKF C1 targets (80 mm / 8 deg at 400 ms; the jig fit scored 163 mm / 36 deg)
   for continuity, but the pass bar is beating A, not the absolute target, until the noise
   floor says the target is reachable.
2. **Cross-fight transfer.** B beats or matches A on 11-45-05 and 14-12-25 at every horizon,
   and no maneuver class regresses more than 20%. This is the real2sim claim: parameters fit
   on Friday practice driving predict Saturday fights.
3. **Noise floor.** From matched-command window pairs (the driver repeats spins, charges, and
   reversals constantly), measure how differently the real robot answers the same command.
   Report every gate relative to that floor. Holdout error at the floor for straights but not
   reversals means a missing model term, not a bad fit.

Report: per-recording, per-maneuver, per-horizon table in
`match_plant_fit_report.md` next to this plan, same shape as `stage4_sim_report.md`.

## Outputs

- `playground/calibration/out/plant_match.toml`, same format and provenance convention as
  `plant_stageA.toml`. Existing files are not overwritten; consumers switch explicitly.
- `nonresponsive_windows.csv` and a contact-window list per recording.
- The report above.

If gate 2 passes, this fit is the candidate for the EKF plant slot and for re-running the
stage 4 nav mapping with match-fit constants.

## Known limits

- Poses are perception output, so this is the perceived plant: command in, perceived pose
  out. That is the correct target for the nav and filter, which consume the same perception,
  but it inherits flat-plane bias and any keypoint scale error.
- Desktop playback frames are affine-warped 2 to 3% versus live Jetson frames. Fit and
  validation tracks all go through the same desktop playback, so the comparison is internally
  consistent, and the fitted gains carry up to that scale error onto the Jetson.
- Battery state is not in these recordings (CRSF battery telemetry logs at debug level), so
  voltage sag over a fight is absorbed into the fitted gains. Per-window gain drift over a
  match is worth plotting in the report as evidence for promoting that telemetry to
  diagnostics.
- Impacts and inverted driving are out of scope by construction of the window gates.

## Next steps

1. Replay the three assigned SVOs with the current model and record the regenerated mcaps.
2. Write the match-window loader and the SVO frame-interval join; validate the join by
   overlaying regenerated poses on original-command timestamps for one known maneuver.
3. Run the ladder fit from the Stage A seed, then the three-plant scoring, and write the
   report.
