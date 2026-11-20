# 2026-11-15 NHRL eval box rate

Measurement behind the `--expected-per-frame` value used with `training/model_eval/conf_sweep.py`. How the sweep works is in `docs/reference/eval/eval_code.md`.

## Result

For NHRL cage footage, `nhrl_keypoints_eval_test` holds 2.81 boxes per frame over its 590 NHRL frames:

| Class | Boxes per frame |
| --- | --- |
| mrs_buff_mk3 | 0.98 |
| opponent | 1.14 |
| house_bot | 0.68 |

The total is below the nominal two robots plus a house bot because the house bot is in frame 68 percent of the time.
