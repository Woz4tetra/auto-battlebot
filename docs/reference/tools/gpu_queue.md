# GPU queue

Design facts behind `training/gpu_queue.py`. Usage is in `CLAUDE.md` ("GPU training queue").

## Estimates

- `MATCH_TIERS` is ordered finest match first, coarsest last. A prediction from three same-shape runs beats one from any five jobs that happen to have run.
- `MIN_HISTORY_SECONDS = 300`: a shorter run is a smoke test or an export, not a training arm to learn from. It is used only when nothing longer exists.

## Waiting for the GPUs

- `gpu_busy_processes` counts compute processes holding real memory, excluding the job's own pids. With `devices`, only processes on those GPU indices count: a tenant pinned to a GPU the job does not use is no reason to wait.
- `wait_for_gpus` refuses to start a job while anything holds its GPUs, whatever the yield hooks did.

## Yield hooks

- Tenant hooks live in `~/.config/gpu_queue/yield.d/` (`YIELD_DIR`), a fixed directory rather than an env var. The worker's environment is whatever the agent that spawned it had (`job_env`), so an env setting would come and go depending on who submitted first.
- `run_yield_hooks` runs every hook with `pause` or `resume`. Output lands in the worker log.
- A pause passes the job's GPUs as `GPU_QUEUE_DEVICES`, and the hook decides whether its tenant is in the way.
- A failing hook is reported and skipped. Because `wait_for_gpus` still gates the start, the worst a broken pause can do is the wait the queue would have had without hooks.

## logs -f

`follow_log` streams a job's log until it ends, then rolls onto whatever runs next. The queue runs one job at a time for hours, so the log path worth watching changes several times over a sweep, and a plain `tail -f` on one path goes quiet when the next arm starts. With an explicit job id (`pinned`) it stops when that job does.

Bytes are copied straight through rather than line-buffered, so the progress bar's carriage returns still redraw in place.

## submit options

- `--work`: job size in any consistent unit, frames x epochs for a training arm. The queue divides what past jobs of the same profile took by their own work to get a rate, so a 58,447-frame arm does not inherit the estimate of an 18,447-frame one. Rates compare only within a profile.
- `--profile`: the cost class a rate is learned within, e.g. `yolo26x-pose@640`. Two jobs share a profile when one unit of work costs them the same. The default is the model and input size read off the command.
- `--eta`: used until the job's own progress takes over. Give it for the first run of a model family, where no measured rate exists and the queue would otherwise say it does not know.
- `logs -f` without a job id rolls onto the next job when one ends, so a whole sweep can be watched with one command. With a job id it stops when that job does.
