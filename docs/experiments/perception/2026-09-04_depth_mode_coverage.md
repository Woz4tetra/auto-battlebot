# Depth cache script details

## Depth mode

- ZED_NEURAL_LIGHT is the `config/_common.toml` default and the cheapest neural mode. NEURAL_PLUS is the most accurate one the SDK offers and is what a depth-based detector deserves to be graded on: on a sample MassD frame it returns depth for 0.718 of pixels against NEURAL_LIGHT's 0.583. `--depth-mode` picks the mode; the cache name records the choice.

## SVO lookup

The SVO for a recording, found by the timestamp suffix the MCAP name carries.

Recordings are named `<profile>_<start>__<svo stamp>.mcap` and the SVO is `<svo stamp>.svo2`, so the suffix after the double underscore is the key. Roots are searched recursively because the May fights live in `svo/NHRL_2026-05-02/` but the 05-01 one is in `svo/tests/`.

## Cache alignment check

Prove the cached depth belongs to the frame the labels describe.

The depth cache reaches each wanted frame with `set_svo_position`. Seeking an SVO can smear the decode, and a smeared seek is silent: the depth still arrives, still has the right shape, and is simply the wrong frame. Every conclusion drawn from a depth gate would then be measuring noise against labels that describe a different moment.

The check re-seeks the same indices and pulls the LEFT image instead of depth, then compares it to the ground-truth PNG that carries the same stamp.

The comparison is relative, not absolute. The pipeline's PNGs are rectified and colour-processed differently from a raw SVO retrieve, and desktop playback frames are affine-warped a few percent against live Jetson frames, so even a perfectly matched pair has a mean absolute difference in the teens. An absolute threshold cannot tell that apart from a wrong frame.

What can: sweep the neighbouring indices too. Whatever constant penalty rectification adds applies to all of them equally, so if the seek lands where it claims, the claimed index is the argmin and its neighbours are worse. A smeared or off-by-N seek puts the argmin somewhere else. The reported number is the fraction of sampled frames whose best offset is 0, plus the margin between the claimed index and its best neighbour.

The comparison is restricted to the ground-truth boxes, dilated. Over a whole frame the arena, walls and crowd are identical from one index to the next and swamp the only thing that moved; the robot is a few percent of the pixels. Inside its own box it is most of them, which is what makes a one-frame offset visible at all.

```
Usage:
    python playground/check_depth_cache_alignment.py training/data/nhrl_keypoints_eval_test         --svo-roots data/svo -o training/data/eval_results/depth_cache_alignment.json
```

- A +-1 index offset counts as aligned: SVO image stamps sit about half a frame before the pipeline stamps that name the GT images, so the true minimum lies between two indices and which side wins is arbitrary. A cache off by more than that is not usable.
