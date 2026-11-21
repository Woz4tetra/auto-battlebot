# Perception reliability

`training/model_eval/perception_reliability.py` measures opponent-track reliability from a playback MCAP recording (a real Jetson SVO replayed in playback, or a live run). It reports how often an opponent track exists and how long the track is lost at a time. The gap-length distribution bounds the horizon the predictor must coast through.

## Source data

- The `perception` diagnostics subsection the control loop emits once per frame: `their_count_live` and `our_present_live`, which count tracks that exist (`src/control_loop/control_loop.cpp`). Recordings made before tracks expired by sigma also carry `their_count_total`, and there "live" meant not stale.
- The `jump_reject` events the FrameId assigner logs.
- Any recording carrying `/diagnostics` (post 2026-06-19) works with no re-instrumentation.
- The message stamp is the camera frame stamp, so dropout durations are real time even under free-run playback.

## Metrics

- Opponent validity: percent of ticks with a live opponent (`their_count_live > 0`).
- Our-robot validity: percent of ticks with a live `OUR_ROBOT_1` (`our_present_live > 0`).
- Opponent dropout gaps: run lengths of consecutive ticks with no live opponent, in frames and ms.
- Our-robot dropout gaps: run lengths of consecutive ticks with no live `OUR_ROBOT_1`. These bound `max_position_sigma_m` (`docs/reference/config/fields.md`). If the gaps are short, holding the "ours" identity across them is cheap.
- Track churn: `jump_reject` events per second (the FrameId assigner refusing a far reassignment).

## Limits

- True track-ID-switch rate is not measured. The diagnostics carry opponent counts, not per-slot identity, so a physical robot swapping FrameId slots is not directly observable. The `jump_reject` rate is the proxy. Exact switch tracking needs the perception diagnostics block to log the live THEIRS FrameId set.

## Usage

```bash
source scripts/activate_python.sh
python training/model_eval/perception_reliability.py \
    data/recordings/auto_battlebot_experiment_playback_2026-06-20_00-15-08.mcap \
    --csv out/perception.csv --plot out/perception.png
```
