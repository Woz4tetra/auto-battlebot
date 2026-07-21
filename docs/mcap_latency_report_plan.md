# Plan: latency report script from MCAP data

## Goal

A script that reads an MCAP recording and produces a latency report — per-stage timing
and end-to-end pipeline latency (mean / p95 / max), plus an optional plot.

## Key finding

Per-stage timing and end-to-end latency are **already recorded** into every MCAP via the
`/diagnostics` topic. No C++ instrumentation is needed — this is purely a read-and-report
script. An existing script, `playground/analyze_nav_diagnostics.py`, already extracts
this data (including pipeline latency mean/p95/max) and is the closest template.

## Where the data comes from

- **Recorder:** `src/mcap_recorder/mcap_recorder.cpp` (mcap C++ lib), ROS1-serialized
  messages. Config `include/mcap_recorder/config.hpp` (`McapRecorderConfig`).
- **Per-stage timers:** `FunctionTimer` (RAII, `include/diagnostics_logger/function_timer.hpp`)
  logs `data["elapsed_ms"]` on destruction. Stages timed in `src/runner.cpp::tick()`:
  `tick` (:322), `camera.get` (:348), `field_filter.track_field` (:387),
  `keypoint_model.update` (:394), `robot_mask_model.update` (:400),
  `robot_filter.update` (:406), `publishers` (:430). Plus loop `rate` (:326-333) and
  **`pipeline/latency_ms`** = `(now - camera frame stamp) * 1000` (:451-460), the true
  end-to-end camera-to-command latency.
- **Wire format:** each publish is a `diagnostic_msgs/DiagnosticArray` on `/diagnostics`.
  Per status: `status.name = subsection` (e.g. `"camera.get"`, `"pipeline"`),
  `status.hardware_id = logger name` (e.g. `"runner"`), values are flattened key/value
  string pairs (`src/diagnostics_logger/ros_diagnostics_backend.cpp:20-40`). A parser keys
  on `(hardware_id, name)` and reads `values["elapsed_ms"]` (or
  `values["latency_ms"]` for `pipeline`).

## Reuse: decode library and template

- **Decoder:** `auto_battlebot/mcap_io.py` — `iter_messages(path, topics=None)` yields
  `(topic, log_time_ns, raw_bytes)`; `decode_diagnostic_array(data)` returns dicts
  `{level, name, message, hardware_id, values: {key: value}}`. `DIAGNOSTICS_TOPIC = "/diagnostics"`.
  Only the `mcap` package is installed (v1.3.1) — use this module, **not** `mcap_ros1`
  (not in venv).
- **Template:** `playground/analyze_nav_diagnostics.py` — argparse (positional `file`,
  `--save PNG`, `--no-show`), `extract_diagnostics(path)` builds one row per
  `message.log_time` keyed on `(hardware_id, name)` into a pandas DataFrame, coerces
  numerics, computes `t = (ts_ns - t0)/1e9`, and already prints `pipeline/latency_ms`
  mean/p95/max.
- **Lint-clean script template:** `scripts/mcap_topic_sizes.py` (argparse `files`
  nargs="+", existence checks, fixed-width text table to stdout).

## Approach

Write `scripts/mcap_latency_report.py` (top-level, lint-clean, reusable). Structure:

1. **CLI** (argparse): positional `file` (Path, existence-checked); `--save PATH` for a
   plot; `--no-show`; optional `--csv PATH` to dump the per-stage table; optional
   `--stages a,b,c` filter.
2. **Extract**: `iter_messages(path, topics=["/diagnostics"])` →
   `decode_diagnostic_array` per message → collect, for each status with an `elapsed_ms`
   (or `pipeline` latency), a row `{t_ns, stage=f"{hardware_id}.{name}", value_ms}`. Load
   into a pandas DataFrame.
3. **Aggregate**: group by stage, compute count, mean, median, p95 (`quantile(0.95)`),
   max, and % of the `runner.tick` mean (stage share of the loop). Include
   `pipeline/latency_ms` and `rate` as their own rows.
4. **Report**: print a fixed-width text table (stage | n | mean | p95 | max | %tick),
   mirroring `mcap_topic_sizes.py` styling. Bordered summary block for the headline
   end-to-end latency (mean / p95 / max) against the 60 ms budget (flag if p95 > 60 ms).
5. **Plot (optional)**: matplotlib timeline of `pipeline/latency_ms` over `t`, plus a
   stacked-bar or box plot of per-stage `elapsed_ms`. `fig.savefig(save_path, dpi=150)`;
   `plt.show()` unless `--no-show`.

## Placement & conventions

- Put in `scripts/` (shebang `#!/usr/bin/env python3`, module docstring, ruff-clean,
  double quotes, line length 100, target py310). `mcap`, `pandas`, `matplotlib`, `numpy`
  are already in `pyproject.toml`.
- Run inside the venv: `source scripts/activate_python.sh` then
  `python scripts/mcap_latency_report.py data/recordings/<file>.mcap`.
- Type-checked: it lives in `scripts/`, so it must pass `venv/bin/mypy scripts/`.

## Testing

- Run against an existing recording under `data/recordings/` and eyeball the table vs.
  `playground/analyze_nav_diagnostics.py`'s pipeline-latency numbers (should match).
- Handle recordings with no `/diagnostics` messages gracefully (clear message, exit
  nonzero).

## Validation

```bash
venv/bin/mypy scripts/
venv/bin/ruff check scripts/mcap_latency_report.py
```

## Open questions

- Output format priority: text table only, or table + plot + CSV? Recommend table by
  default, plot/CSV behind flags.
- Whether to also break down non-runner loggers (e.g. `yolo_keypoint_model`,
  `ros_publisher`) that emit their own `elapsed_ms`. Recommend including all
  `(hardware_id, name)` pairs that carry `elapsed_ms`, sorted by mean descending.
