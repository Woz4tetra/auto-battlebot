# training

`training/` is standalone scripts and is deliberately not a package. Code that `scripts/` or
`simulation/` needs from here moves to `auto_battlebot/` first.

## GPU training queue

Several agents share the three A6000s. Every training arm runs DDP across all three, so
the box runs one job at a time. Do not launch training directly; submit it:

```bash
# --work is frames x epochs (25,914 x 100 here). The queue learns seconds per unit within
# a profile, so an arm's size and model both reach the estimate.
venv/bin/python training/gpu_queue.py submit --name B_s384x640 --by <agent> \
  --work 2591400 --profile yolo26s@640 -- \
  venv/bin/python training/yolo/train.py training/data/nhrl_robots_bbox_2class yolo26s \
  -d 0 1 2 -b 96 -e 100
venv/bin/python training/gpu_queue.py status          # --json for parsing
venv/bin/python training/gpu_queue.py logs 3 --tail 40
venv/bin/python training/gpu_queue.py logs -f         # follow the running job, rolling
                                                      # onto the next when it finishes
```

Pass `--work` and `--profile` on every training submit: without them the queue falls back to
what past jobs of the same shape took, which is how a `yolo26x-pose` arm inherited 2h18m from
sixteen `yolo26s-pose` runs. A rate is never borrowed across models, so the first run of a
model family has no history to learn from; give it `--eta 12h` and the queue uses that until
the job's own progress takes over.

`submit` starts the worker if none is running and sets `NCCL_P2P_DISABLE=1` for
multi-GPU jobs. The worker waits for the GPUs to go idle before each job, so a run
started outside the queue delays it rather than colliding with it. Check `status`
before submitting, and never kill a job you did not submit; use `cancel <id>`.

State and logs live in `runs/queue/` (gitignored). Jobs run in submission order;
`status` lists them in that order with an estimated finish time for each, taken from
the running job's own progress and from what past jobs took.
