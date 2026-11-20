# GPU queue

Design facts behind `auto_battlebot/gpu_queue.py`. Usage is in `CLAUDE.md` ("GPU training queue").

## Estimates

- `MATCH_TIERS` is ordered finest match first, coarsest last. A prediction from three same-shape runs beats one from any five jobs that happen to have run.
- `MIN_HISTORY_SECONDS = 300`: a shorter run is a smoke test or an export, not a training arm to learn from. It is used only when nothing longer exists.
- `forecast` trusts past runs over a job's own progress until `LOG_PROGRESS_TRUST_FRACTION`. The first epochs carry the dataset scan and warmup, so extrapolating from them overshoots badly: `F_s_stretch` read 3h13m at epoch 3 against the 1h53m its sibling arm took. A job with no history is trusted once it passes 2% progress.
- `matching_history` returns nothing, and the queue says "no comparable runs yet", when no earlier job ran the same script. Averaging in unrelated jobs caused the wild misses: `bench_geometry`, ten seconds long, inherited 1h53m from the training arms.
- `usable_history` drops minute-long jobs when the job being predicted is a real training run. `smoke_rect` ran one epoch in a minute, nearly all of it startup, so its seconds per epoch was nothing like a real arm's. It was the only other `yolo26n@640` run at the time and dragged the `A2` estimate 30 minutes low.
- `job_shape` defines comparable jobs: same script, model, input size, and dataset. An `s` model costs about a third more per epoch than an `n` at the same input size, and 1024 costs more than 640. Pooling them all gave `F_s_stretch` an estimate 34 minutes short. Matching on shape lets each finished job sharpen the next prediction instead of blurring it.

## Waiting for the GPUs

- `gpu_busy_processes` counts compute processes holding real memory, excluding the job's own pids. With `devices`, only processes on those GPU indices count: a tenant pinned to a GPU the job does not use is no reason to wait.
- `BUSY_MIB` (`AB_GPU_QUEUE_BUSY_MIB`, default 1024) is the memory a compute process must hold to count as somebody using the GPUs. Sunshine's desktop streamer sits at about 260 MiB and must not count as busy.
- `process_ancestry` returns a pid and everything that spawned it. A job owns its GPU processes at a distance: Ultralytics re-launches itself under `torchrun`, which puts one child per GPU two levels below the pid the queue recorded.
- `wait_for_gpus` refuses to start a job while anything holds its GPUs, whatever the yield hooks did.

## Yield hooks

- Tenant hooks live in `~/.config/gpu_queue/yield.d/` (`YIELD_DIR`), a fixed directory rather than an env var. The worker's environment is whatever the agent that spawned it had (`job_env`), so an env setting would come and go depending on who submitted first.
- `run_yield_hooks` runs every hook with `pause` or `resume`. Output lands in the worker log.
- A pause passes the job's GPUs as `GPU_QUEUE_DEVICES`, and the hook decides whether its tenant is in the way.
- `HOOK_TIMEOUT = 300` s, because a pause can sit through a container's full stop grace period (60 s for vLLM).
- megamind's model server is one tenant. Link its script into the hook directory with `ln -s ~/home-apps/megamind/extractor.sh ~/.config/gpu_queue/yield.d/megamind`. Hooks run with `pause` before each job and with `resume` once the queue has sat idle for `YIELD_RESUME_IDLE_SECONDS` (600 s).
- A failing hook is reported and skipped. Because `wait_for_gpus` still gates the start, the worst a broken pause can do is the wait the queue would have had without hooks.

## logs -f

`follow_log` streams a job's log until it ends, then rolls onto whatever runs next. The queue runs one job at a time for hours, so the log path worth watching changes several times over a sweep, and a plain `tail -f` on one path goes quiet when the next arm starts. With an explicit job id (`pinned`) it stops when that job does.

Bytes are copied straight through rather than line-buffered, so the progress bar's carriage returns still redraw in place.

## submit options

- `--work`: job size in any consistent unit, frames x epochs for a training arm. The queue divides what past jobs of the same profile took by their own work to get a rate, so a 58,447-frame arm does not inherit the estimate of an 18,447-frame one. Rates compare only within a profile.
- `--profile`: the cost class a rate is learned within, e.g. `yolo26x-pose@640`. Two jobs share a profile when one unit of work costs them the same. The default is the model and input size read off the command.
- `--eta`: used until the job's own progress takes over. Give it for the first run of a model family, where no measured rate exists and the queue would otherwise say it does not know.
- `logs -f` without a job id rolls onto the next job when one ends, so a whole sweep can be watched with one command. With a job id it stops when that job does.

## Workarounds

- **`NCCL_P2P_DISABLED_ENV`.** `submit` sets `NCCL_P2P_DISABLE=1` for any job on more than one GPU. NCCL peer-to-peer is broken on megamind's three A6000s, and DDP hangs silently without it. Unverified: nobody has rerun a multi-GPU job without it.
