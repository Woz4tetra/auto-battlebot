# gpu queue

Notes moved out of code comments during the comment purge. Each entry names the file and symbol it came from.

## Notes moved from code comments (2026-11-15)

- `training/gpu_queue.py`, `follow_log`:

  ```text
  Stream a job's log until it ends, then roll onto whatever runs next.

  The rollover is the point. The queue runs one job at a time for hours, so the
  log path worth watching changes several times over a sweep, and a plain
  `tail -f` on one path goes quiet exactly when the next arm starts. With an
  explicit job id (`pinned`) it stops when that job does instead.

  Bytes are copied straight through rather than line-buffered, so the progress
  bar's carriage returns still redraw in place.
  ```

- `training/gpu_queue.py` (`above MATCH_TIERS = (`):
  Coarsest match last: a prediction from three same-shape runs beats one from any five jobs that happen to have run.
- `training/gpu_queue.py`, `run_yield_hooks`:
  Run every tenant hook with `pause` or `resume`; output lands in the worker log.
  A pause passes the job's GPUs as GPU_QUEUE_DEVICES, and the hook decides whether its tenant is in the way. A failing hook is reported and skipped. wait_for_gpus still refuses to start a job while anything holds its GPUs, so the worst a broken pause can do is the wait the queue would have had without hooks.
- `training/gpu_queue.py`, `gpu_busy_processes`:
  Compute processes holding real memory, excluding our own job's pids.
  With `devices`, only processes on those GPU indices count: a tenant pinned to a GPU the job does not use is no reason to wait.
- `training/gpu_queue.py` (`above MIN_HISTORY_SECONDS = 300`):
  A run this short is a smoke test or an export, not a training arm to learn from.
- `training/gpu_queue.py` (`above YIELD_DIR = Path.home() / ".config" / "gpu_queue" / "yield.d"`):
  Tenant hooks live in a fixed directory, not an env var: the worker's environment is whatever the agent that spawned it had (see job_env), so an env setting would come and go depending on who submitted first.
