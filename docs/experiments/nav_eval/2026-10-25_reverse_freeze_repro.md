# 2026-10-25 reverse freeze repro

## Notes moved from code comments (2026-11-15)

### `playground/nav_eval/repro_1025.py`

On 10/25 the policy's strong reverse commands reached the wheels as about -12% duty. Two stages
on the field's command path could explain it, and the sim has neither: the host transmitter's
linear slew limit and the firmware motor guard's scale. This runs `rl_nav_p4` in lockstep in four
arms on the same scenarios (slew and guard, slew alone, guard alone, neither) and scores each arm
with the statistic measured on the field recordings.

The statistic: a reverse window is a run of policy ticks where the trailing 0.5 s mean of the
policy's linear command is below -0.5, lasting at least 0.5 s, with autonomy on. The number is
the median wheel command (mean of the firmware's left and right commands, percent, positive
forward) over every sample inside those windows, pooled. The CI resamples windows.

source scripts/activate_python.sh
# the field number, from the three 10/25 recordings
python playground/nav_eval/repro_1025.py field --out-dir /tmp/repro_1025
# export the evaluation scenarios once, then run the four arms on them
python playground/rl/lockstep_eval.py export --count 128 --seed 2026 \
--out /tmp/repro_1025/scenarios.json
python playground/nav_eval/repro_1025.py sim /tmp/repro_1025/scenarios.json \
--out-dir /tmp/repro_1025
# the four-arm table against the field number
python playground/nav_eval/repro_1025.py report --out-dir /tmp/repro_1025

- config/mr_stabs_mk2_fid_zed_box.toml on 10/25: max_motor_rpm_per_sec 2000 over max_motor_rpm 1500. The guard scale is the median inside the field's reverse windows. (at `FIELD_SLEW_PER_S = 2000.0 / 1500.0`)
- Chatter: the commanded linear's mean change per policy tick. A slew limiter only holds the sent command back while the command keeps moving. (at `"chatter": float(np.mean(np.concatenate([np.abs(np.diff(c)) for c in w`)

