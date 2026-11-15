# 2026-09-04 depth mode coverage

## Notes moved from code comments (2026-11-15)

### `playground/depth_cache/cache_gt_depth.py`

- ZED_NEURAL_LIGHT is the config/_common.toml default and the cheapest neural mode. NEURAL_PLUS is the most accurate one the SDK offers and is what a depth-based detector deserves to be graded on: on a sample MassD frame it returns depth for 0.718 of pixels against NEURAL_LIGHT's 0.583. Pass --depth-mode to pick; the cache name records the choice. (at `DEFAULT_DEPTH_MODE = "NEURAL_PLUS"`)
- config/_common.toml does not set depth_minimum_distance; 0.3 m is the ZED default for METER units and the arena floor is never closer than that. (at `DEPTH_MIN_M = 0.3`)

Recordings are named `<profile>_<start>__<svo stamp>.mcap` and the SVO is `<svo stamp>.svo2`,
so the suffix after the double underscore is the key. Roots are searched recursively because
the May fights live in `svo/NHRL_2026-05-02/` but the 05-01 one is in `svo/tests/`.

### `playground/depth_cache/check_depth_cache_alignment.py`

`cache_gt_depth.py` reaches each wanted frame with `set_svo_position`. Seeking an SVO can smear
the decode, and a smeared seek is silent: the depth still arrives, still has the right shape, and
is simply the wrong frame. Every conclusion drawn from a depth gate would then be measuring
noise against labels that describe a different moment.

The check re-seeks the same indices and pulls the LEFT image instead of depth, then compares it to
the ground-truth PNG that carries the same stamp.

The comparison is relative, not absolute. The pipeline's PNGs are rectified and colour-processed
differently from a raw SVO retrieve, and desktop playback frames are affine-warped a few percent
against live Jetson frames, so even a perfectly matched pair has a mean absolute difference in the
teens. An absolute threshold cannot tell that apart from a wrong frame.

What can: sweep the neighbouring indices too. Whatever constant penalty rectification adds applies
to all of them equally, so if the seek lands where it claims, the claimed index is the argmin and
its neighbours are worse. A smeared or off-by-N seek puts the argmin somewhere else. The reported
number is the fraction of sampled frames whose best offset is 0, plus the margin between the
claimed index and its best neighbour.

The comparison is restricted to the ground-truth boxes, dilated. Over a whole frame the arena,
walls and crowd are identical from one index to the next and swamp the only thing that moved; the
robot is a few percent of the pixels. Inside its own box it is most of them, which is what makes a
one-frame offset visible at all.

Usage:
python playground/check_depth_cache_alignment.py training/data/nhrl_keypoints_eval_test         --svo-roots data/svo -o training/data/eval_results/depth_cache_alignment.json

- Pixels of slack around each GT box, so a robot that moved slightly still falls inside the window being compared rather than half outside it. (at `BOX_PAD_PX = 20`)
- Per-frame argmin is a weak matcher: when a robot barely moves between adjacent indices, the winner among neighbours is decided by noise. Averaging the curve over every sampled frame is the statistic that actually separates a good seek from a bad one. A correct seek gives a V centred on 0; a broken one gives a flat curve, because then no index is special. (at `curve = {`)
- How much better the claimed index is than the distant ones. A flat curve means the seek carries no frame-specific information and the cache cannot be trusted. (at `"curve_depth": round(float(np.mean(far)) - curve[0], 2) if far else No`)
- +-1 counts as aligned: SVO image stamps sit about half a frame before the pipeline stamps that name the GT images, so the true minimum lies between two indices and which side wins is arbitrary. A cache off by more than that is not usable. (at `aligned = abs(result["curve_argmin"]) <= 1 and (result["curve_depth"] `)

