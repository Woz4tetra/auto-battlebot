# Gate clips: what each false-positive gate is actually looking at

One clip per eval recording, from `playground/make_gate_clips.py`. The scores in
`../logo_false_positive_2026-09-03.md` say which gate wins on which arena. These say why.

Regenerate:

```bash
python playground/make_gate_clips.py --frames 300 --box-rgb-fraction 0.45 \
    --predictions training/data/eval_results/logo_fp_mixed \
    -o docs/experiments/perception_performance/clips_logo_fp
```

The mp4s are gitignored (`*.mp4`), so they live here next to the report but are not committed.
Rerun the command to get them back.

## Reading a frame

Four panels, all showing the same detector boxes from the deployed engine
(`bbox_2class_mixed_2026-07-31`, conf 0.6):

| panel | shows | box colour |
|---|---|---|
| top left | the frame | white = ground truth, yellow = every detector box, labelled `fg / height / prominence` |
| top right | background-subtraction foreground | green kept, red dropped by the RGB gate at 0.45 |
| bottom left | height above the fitted field plane | green kept, red dropped by the height gate at 0.30, contour = inside the 0.02-0.10 m band |
| bottom right | depth prominence | green kept, red dropped by the prominence gate at 0.15, contour = above 0.20 m |

Each gate panel's header carries its own kept/dropped count for that frame.

Two things to know before reading them:

- **Ground truth is sparse.** Labels exist on roughly every 60th frame, so most frames have no
  white boxes. The clip runs consecutive frames because that is what shows the gates behaving;
  it is not a scored sequence.
- **Each clip starts 60 frames before the labelled frame carrying the most false positives**, so
  the interesting failure is near the start rather than somewhere in the middle.

## What to look for

**MassD 2026-08-29 is the arena with the logo.** The "MASSACHUSETTS RESURGENCE" banner lying flat
on the plywood draws a persistent, confident detector box. Watch the three gate panels disagree
about it:

- The **height** panel drops it. The banner sits on the plane, so it falls below the band.
- The **prominence** panel keeps it, brightly. The banner lies against the arena wall, and
  prominence is a local max filter, so the window reaches over the wall to the room beyond and a
  flat graphic reads as maximally proud of its surroundings. This is the AUC 0.437 result made
  visible: the signal prefers exactly the box it should reject.
- The **RGB** panel drops it once the threshold is high enough. The banner has been part of the
  floor median since the field init, so it produces no foreground.

**The May NHRL recordings fail the other way.** The cage is dark, the floor is dark and covered in
confetti, and the robots are dark. The RGB panel goes faint over real robots and the gate starts
dropping boxes it should keep, which is the 0.788 to 0.082 recall collapse in the report. The
height panel holds up better here, because a robot is 3 cm tall regardless of how well it contrasts
with the floor.

Watch the top-right panel in a May clip and a MassD clip back to back. That contrast is the whole
argument against shipping a fixed foreground threshold.

## Per-recording notes

Kept counts are over the whole 300-frame window, out of every detector box in it. They are not
accuracy: a gate that keeps everything and a gate that keeps only the right things both look
"high" here. Read them against the scored numbers in the report.

| recording | start frame | boxes | RGB kept | height kept | prominence kept |
|---|---|---|---|---|---|
| `main_2026-05-01_17-42-20` | 0 | - | - | - | - |
| `main_2026-05-02_10-06-02` | 7940 | 740 | 12 (2%) | 249 (34%) | 740 (**100%**) |
| `main_2026-05-02_11-45-05` | 15299 | 808 | 49 (6%) | 115 (14%) | 612 (76%) |
| `main_2026-05-02_14-12-25` | 1480 | 899 | 382 (43%) | 386 (43%) | 897 (**100%**) |
| `main_2026-05-02_15-35-00` | 665 | 610 | **0 (0%)** | 353 (58%) | 610 (**100%**) |
| `main_2026-05-02_16-18-05` | 3088 | 833 | 8 (1%) | 704 (85%) | 833 (**100%**) |
| `main_2026-05-02_17-26-12` | 6340 | 739 | 140 (19%) | 591 (80%) | 720 (97%) |
| `mrs_buff_mk3_massd_2026-08-29` | 58 | 773 | 545 (**70%**) | 675 (87%) | 773 (**100%**) |

Three things fall out of that column set:

- **Prominence keeps essentially everything, everywhere.** 100% on five of seven recordings and
  never below 76%. As a gate it barely fires, and when it does it is not firing on the boxes that
  deserve it. This is the AUC 0.437 result in a different form.
- **The RGB gate's keep rate is the portability problem, in one column.** 70% on MassD against 0%
  to 19% on five of the six May recordings, at one fixed threshold of 0.45. The same physical test
  means something different in each venue.
- **It varies between fights at the same venue too.** 14-12-25 keeps 43% where 15-35-00 keeps 0%,
  both at the NHRL cage. So this is not simply "MassD versus NHRL"; per-arena calibration would not
  have been enough either.

The 05-01 recording rendered before the per-box counters were added, so its row has no totals. The
clip itself is complete and readable; re-render it with `--skip-existing` omitted if the numbers
are wanted.

## Rendering notes

Render each recording in its own process. The ZED SDK does not fully release GPU memory across
`open`/`close` cycles, so the third camera open inside one process dies without a catchable
exception, after two clips have already been written. Batch it with a shell loop rather than the
script's own `--subdataset`-less mode:

```bash
for sub in training/data/nhrl_keypoints_eval_test/*/; do
  venv/bin/python -u playground/make_gate_clips.py --subdataset "$(basename "$sub")" \
    --frames 300 --box-rgb-fraction 0.45 \
    --predictions training/data/eval_results/logo_fp_mixed \
    -o docs/experiments/perception_performance/clips_logo_fp
done
```
