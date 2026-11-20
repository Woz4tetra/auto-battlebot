# Recorder test seam and DeepLab rollout

Follow-ups from the 2026-11-19 bug fixes on `benw/nav-v2-phase0`. Do them in this order: step 1
protects the recordings that steps 2 and 3 produce.

## 1. Stop the C++ tests deleting real recordings

- Bug: `McapRecorderToggleTest.HoldingDirectoryScanSurvivesAnUnreadableEntry` runs the recorder on
  the real `data/recordings` with a 1 GB holding limit, so every full `auto_battlebot_test` run
  deletes the oldest `.mcap` files there until the directory fits. On 2026-11-19 it deleted about
  283 test MCAPs and 10 replay MCAPs on the dev box. The toggle tests in `test_runner_commands.cpp`
  also leave `toggle_*_test` MCAPs there.
- Cause: `src/mcap_recorder/mcap_recorder.cpp` builds the output path from
  `std::filesystem::current_path() / "data" / "recordings"`, with no override.
- Fix: a recordings directory field in `McapRecorderConfiguration`, defaulted to the current path,
  passed in through the constructor. The recorder and runner tests point it at a per-test temp
  dir.
- Done when: a full C++ run leaves `data/recordings` byte-identical (compare `ls -l` before and
  after with a 1 GB+ dummy file in place), and the ledger entry under "Open, found 2026-11-19" in
  `docs/plans/comment_purge_hack_ledger.md` moves to Fixed.
- Until then: never run the C++ suite on a Jetson, and copy replay MCAPs out of
  `data/recordings` before a full run on the dev box.

## 2. Confirm the DeepLab fix on the Jetson

Report: `docs/experiments/perception/2026-11-19_deeplab_preprocessing_fix.md`. The x86 TensorRT
replay matched the `rgb, dataset, deployed` arm at 0.990 IoU. The Jetson builds its own aarch64
engine, which is unchecked.

1. Deploy `benw/nav-v2-phase0` to the Jetson and rebuild. The engine itself does not change; the
   C++ preprocessing does.
2. Record a short session with field inits (human-driven or stationary; no autonomous drive).
3. Copy the MCAP to the dev box and run:

   ```bash
   source scripts/activate_python.sh
   python -m playground.deeplab_preprocess.recorded_mask_agreement <mcap> \
     --checkpoint <deeplab checkpoint>
   ```

4. Pass: recorded masks match the `rgb, *` arms better than the `bgr, *` arms. The old Jetson
   recordings gave BGR 0.880 against RGB 0.822 to 0.827, so the fixed build should flip that.
5. Add the result to the report as a dated section.

## 3. Settle the field-fit result

The fixed build moved the fitted MassD field side from 2.32 m to 2.09 m against a measured 2.26 m
(mean absolute side error 0.199 to 0.174 m), from one init on each of 6 SVOs. That is too few to
call a regression or a gain.

- Replay each MassD SVO with several `svo_start_frame` values (say 5 per SVO, 30 inits per build),
  old and fixed build, x86.
- Pin `svo_start_frame` in each scratch overlay and delete `config/_replay_scratch.toml` after.
- Score fitted side length and its spread per build against 2.26 m.
- If the fixed build is consistently small, check whether the tighter mask trims the floor edge;
  the BF1@2 numbers in the report are the place to start.
- Needs step 1 first, so the replay MCAPs survive a test run.

## 4. Comment purge phase 5

Fix `HACK` ledger rows one at a time, per `docs/plans/comment_purge_and_config_clarity_plan.md`
(Lock-down). Pilot one batch first and record its cost in the plan's Pilot record before fanning
out. The cheapest rows are "Claims that failed verification" in the ledger.
