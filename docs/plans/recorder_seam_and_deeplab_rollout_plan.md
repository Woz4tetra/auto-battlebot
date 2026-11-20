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
  `docs/plans/workaround_ledger.md` moves to Fixed.
- Until then: never run the C++ suite on a Jetson, and copy replay MCAPs out of
  `data/recordings` before a full run on the dev box.

## 2. Confirm the DeepLab fix on the Jetson

Done 2026-11-20 on auto-battlebot-compute-1 by SVO playback: recorded masks match
`rgb, dataset, deployed` at 0.9949 IoU against 0.9122 for the closest BGR arm
(`docs/experiments/perception/2026-11-19_deeplab_preprocessing_fix.md`, "Jetson engine check").
A live e-CAM25 session is still unchecked.

## 3. Settle the field-fit result

Done 2026-11-20: 30 inits per build. On converged good-view inits the fix moves side bias from
+0.146 m to -0.038 m. Half the good-view inits fail in both builds, so the field fit itself is the
next problem (report section "Field fit over 30 inits").

## 4. Comment purge phase 5

Done 2026-11-20: 0 counted comment lines, baseline `{}`. Workarounds that still need a root-cause
fix are in `docs/plans/workaround_ledger.md`; fix them one at a time. The cheapest rows are "Claims
that failed verification".
