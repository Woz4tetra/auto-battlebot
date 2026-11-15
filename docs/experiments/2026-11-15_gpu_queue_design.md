# 2026-11-15 gpu queue design

Notes moved out of code comments during the comment purge. Each entry names the file and symbol it came from.

## Notes moved from code comments (2026-11-15)

- `training/gpu_queue.py`, `forecast` (`above if fraction >= LOG_PROGRESS_TRUST_FRACTION or (history is None and fraction > 0.02):`):
  The first epochs carry the dataset scan and warmup, so extrapolating from them overshoots badly: F_s_stretch read 3h13m at epoch 3, against the 1h53m its sibling arm took. Past runs win until a job's own rate has settled.
- `training/gpu_queue.py`, `matching_history` (`above return [], "no comparable runs yet"`):
  Nothing has run this script before. Averaging in unrelated jobs is where the wild misses came from, bench_geometry, ten seconds long, inherited 1h53m from the training arms, so the queue says it does not know instead.
- `training/gpu_queue.py`, `usable_history`:
  Finished jobs worth learning from when predicting this one.
  smoke_rect ran one epoch in a minute, nearly all of it startup, so its seconds-per-epoch is nothing like a real arm's, and being the only other yolo26n @640 run at the time, it dragged A2's estimate 30 minutes low. When the job being predicted is a real training run, the minute-long jobs are dropped.
- `training/gpu_queue.py`, `job_shape`:
  What makes two jobs comparable: same script, model, input size, dataset.
  An `s` model costs about a third more per epoch than an `n` at the same input size, and 1024 costs more than 640, so pooling them all gave F_s_stretch an estimate 34 minutes short. Matching on the shape is what makes each finished job sharpen the next prediction instead of blurring it.
- `training/gpu_queue.py`, `process_ancestry`:
  A pid and everything that spawned it.
  A job owns its GPU processes at a distance: Ultralytics re-launches itself under torchrun, which puts one child per GPU two levels below the pid the queue recorded.
- `training/gpu_queue.py` (`above HOOK_TIMEOUT = 300`):
  A pause can sit through a container's full stop grace period (60 s for vLLM).
- `training/gpu_queue.py` (`above BUSY_MIB = int(os.environ.get("AB_GPU_QUEUE_BUSY_MIB", "1024"))`):
  A compute process holding more than this is treated as somebody using the GPUs. Sunshine's desktop streamer sits at ~260 MiB and must not count as busy.
- `training/gpu_queue.py`:

  ```text
  Serial GPU job queue, so several agents can share megamind's three A6000s.

  Every training arm in the experiment plans runs DDP across all three GPUs, so the
  scheduling unit is the whole box: one job at a time, in submission order. Agents
  submit and poll rather than launching training directly.

      training/gpu_queue.py submit --name B_s384x640 --work 2591400 --profile yolo26s@640 -- \
          venv/bin/python training/yolo/train.py training/data/... yolo26s -d 0 1 2 -b 96
      training/gpu_queue.py status
      training/gpu_queue.py logs 3 --tail 40
      training/gpu_queue.py logs -f            # follow whatever is training, across jobs

  Finish times are learned from what past jobs took, so tell the queue how big a job is:
  `--work` (frames times epochs, for a training arm) and `--profile` (the cost class the
  seconds-per-unit rate is learned within). Neither is guessable from the command line, where
  epochs often arrive positionally and the frame count lives inside a dataset. A rate is never
  borrowed across models, since an `x` costs several times an `s` per unit, so the first run of
  a model family has nothing to learn from and should carry `--eta` instead.

  A worker process pops jobs; `submit` starts one if none is alive. The worker waits
  for the GPUs to go idle before each job, so a run launched outside the queue (by
  hand or by an agent that has not adopted it) delays the queue instead of colliding
  with it.

  Long-lived GPU tenants step aside while the queue has work. Every executable in
  ~/.config/gpu_queue/yield.d/ runs with `pause` before each job, and with `resume`
  once the queue has sat empty for the idle timeout. The job's devices arrive as
  GPU_QUEUE_DEVICES (e.g. "0,1"), so a tenant pinned to a GPU the job leaves alone can
  keep running. megamind's model server is one:

      ln -s ~/home-apps/megamind/extractor.sh ~/.config/gpu_queue/yield.d/megamind
  ```

