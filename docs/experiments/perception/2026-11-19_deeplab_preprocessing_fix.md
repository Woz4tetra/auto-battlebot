# Does fixing the C++ DeepLab preprocessing improve the field mask?

Yes. Feeding the network RGB instead of BGR raises macro IoU from 0.9219 to 0.9440 (+0.0221, 95% CI
+0.0207..+0.0235) on the 5,973-frame held-out set, and the worst field from 0.7889 to 0.8576. The
normalization bug the audit found (ImageNet stats instead of the corpus stats) is not what costs
accuracy: on its own it moves macro IoU by +0.0004. The fix ships both, with the C++ resize and pad
left as they were.

## Findings

1. **Channel order is the bug that matters.** The C++ path converted only 4-channel frames to RGB.
   Every camera delivers 3-channel BGR (`zed_device.cpp`, `zed_one_rgb_camera.cpp`,
   `v4l2_rgb_camera.cpp`, `video_decoder.cpp`), so the deployed network saw BGR. The YOLO path
   swaps (`kSwapRbToRgb`); the DeepLab path did not. Swapping to RGB: macro IoU +0.0218, worst
   field +0.0686, boundary F1 at 10 px +0.0586, all with intervals clear of zero.
2. **Normalization alone is noise-sized.** ImageNet against corpus stats with BGR held fixed: macro
   IoU +0.0004, global IoU -0.0002, BF1@2 -0.0034. With RGB: macro IoU 0.9437 against 0.9440. The
   two mean vectors differ by 0.02 to 0.03 per channel, and the network absorbs that.
3. **The robot's own camera views gain the most.** `own_arena` IoU 0.8375 to 0.9237 (+0.0862,
   +0.0812..+0.0907), `own_room_floor` 0.7889 to 0.8598 (+0.0709), `cage4` +0.0508. Two broadcast
   fields lose a little: `cage2` -0.0007 and `cage5` -0.0009. `skycam` is not significant.
4. **The C++ geometry stays.** Training resizes with nearest neighbour then zero-pads; C++
   replicate-pads at frame resolution then resizes bilinearly. With RGB and corpus stats, the
   training geometry gains +0.0024 macro IoU but loses 0.0584 BF1@2 (0.3984 against 0.4568): its
   nearest-neighbour downsample makes blocky edges at full resolution. Global IoU also favours the
   C++ geometry (0.9483 against 0.9466).
5. **The old path's mask runs up the walls.** On 372 frames sampled from the MassD and NHRL Jetson
   recordings, the BGR path marks 28.7% of the frame as floor and the RGB path 28.4%. On the 39
   frames where the robot initialized its field, 29.5% against 28.0%. The extra floor sits along the
   far rails and the back wall (`assets/deeplab_preprocessing_2026-11-19/massd_overlay_before_after.jpg`,
   red is old-only).
6. **Downstream, the field fit is mixed on 6 replays.** Old against fixed build on six MassD SVOs:
   mean absolute error of the fitted field side against the measured 2.26 m floor goes from 0.20 m
   to 0.17 m, but the bias flips from 0.06 m too large to 0.17 m too small. Six inits is too few to
   call it.

## Setup

| | |
| --- | --- |
| Model | `data/models/field_deeplabv3p_r50_2026-07-29.pth` (DeepLabv3+ ResNet-50, `image_size` 344, `pad_size` 20, input 384) |
| Eval set | `/media/storage/auto-battlebots-archive/deeplab_field_2026-07-29/val` on megamind: 6,001 frames, 5,973 after dropping `unclassified`, 13 fields, scene-disjoint from train |
| Arms | 2 x 2 x 2: channel order (RGB, BGR) x normalization (corpus, ImageNet) x geometry (training, deployed) |
| Before | `bgr, imagenet, deployed`: the C++ path at `1cc911fb` |
| After | `rgb, dataset, deployed`: the C++ path after this fix |
| Scoring | Native frame resolution (1920x1080 for 95% of frames), IoU and boundary F1 at 2/5/10 px, macro over fields, paired bootstrap 1000x against the before arm |
| Hardware | Laptop RTX 4080, PyTorch checkpoint |

```bash
CK=data/models/field_deeplabv3p_r50_2026-07-29.pth
ARGS=""
for c in rgb bgr; do for n in dataset imagenet; do for g in training deployed; do
  ARGS="$ARGS --candidate ${c}_${n}_${g}=$CK --preprocess ${c}_${n}_${g}=$c,$n,$g"
done; done; done
venv/bin/python training/deeplab/score_masks.py <corpus>/val --field-index <corpus>/field_index.json \
  $ARGS --baseline bgr_imagenet_deployed --by-field --score-resolution native

venv/bin/python -m playground.deeplab_preprocess.recorded_mask_agreement \
  'data/saved_recordings/MassD_2026-08-29/*.mcap' 'data/saved_recordings/NHRL_2026-05-02/*.mcap' \
  --checkpoint $CK --every 300
```

## Results

| Arm (channels, normalization, geometry) | Macro IoU | Worst-field IoU | Global IoU | Macro BF1@2 | Macro BF1@5 | Macro BF1@10 |
| --- | --- | --- | --- | --- | --- | --- |
| `bgr, imagenet, deployed` (before) | 0.9219 | 0.7889 | 0.9337 | 0.4260 | 0.6022 | 0.6839 |
| `bgr, dataset, deployed` | 0.9223 | 0.7908 | 0.9335 | 0.4226 | 0.5994 | 0.6828 |
| `rgb, imagenet, deployed` | 0.9437 | 0.8575 | 0.9481 | 0.4587 | 0.6643 | 0.7425 |
| `rgb, dataset, deployed` (after) | 0.9440 | 0.8576 | 0.9483 | 0.4568 | 0.6639 | 0.7430 |
| `bgr, imagenet, training` | 0.9294 | 0.8089 | 0.9371 | 0.3541 | 0.6045 | 0.7072 |
| `bgr, dataset, training` | 0.9296 | 0.8099 | 0.9369 | 0.3542 | 0.6034 | 0.7071 |
| `rgb, imagenet, training` | 0.9462 | 0.8574 | 0.9465 | 0.3969 | 0.6533 | 0.7520 |
| `rgb, dataset, training` | 0.9464 | 0.8574 | 0.9466 | 0.3984 | 0.6544 | 0.7531 |

After against before, paired bootstrap:

| Metric | Macro | Worst field | Global |
| --- | --- | --- | --- |
| IoU | +0.0221 (+0.0207..+0.0235) | +0.0687 (+0.0382..+0.0773) | +0.0146 (+0.0138..+0.0155) |
| BF1@2 | +0.0309 (+0.0280..+0.0338) | +0.0203 (+0.0174..+0.0231) | +0.0222 (+0.0202..+0.0244) |
| BF1@10 | +0.0592 (+0.0567..+0.0617) | +0.0802 (+0.0716..+0.0885) | +0.0481 (+0.0466..+0.0500) |

### By field, IoU

| Field | Frames | Before | `rgb, imagenet, deployed` | After | `rgb, dataset, training` | After minus before |
| --- | --- | --- | --- | --- | --- | --- |
| cage2 | 987 | 0.9891 | 0.9885 | 0.9885 | 0.9906 | -0.0007 (-0.0008..-0.0005) |
| cage4 | 76 | 0.9354 | 0.9863 | 0.9863 | 0.9873 | +0.0508 (+0.0482..+0.0530) |
| cage5 | 223 | 0.9911 | 0.9903 | 0.9902 | 0.9801 | -0.0009 (-0.0012..-0.0006) |
| cage6 | 1684 | 0.9370 | 0.9485 | 0.9487 | 0.9429 | +0.0117 (+0.0102..+0.0134) |
| cage7 | 66 | 0.9812 | 0.9914 | 0.9913 | 0.9872 | +0.0101 (+0.0074..+0.0132) |
| mini_bot | 444 | 0.9755 | 0.9819 | 0.9819 | 0.9798 | +0.0065 (+0.0053..+0.0076) |
| mobilecam | 337 | 0.8503 | 0.8575 | 0.8576 | 0.8574 | +0.0073 (+0.0045..+0.0102) |
| own_arena | 370 | 0.8375 | 0.9239 | 0.9237 | 0.9231 | +0.0862 (+0.0812..+0.0907) |
| own_practice_table | 260 | 0.9094 | 0.9219 | 0.9239 | 0.9569 | +0.0144 (+0.0006..+0.0292) |
| own_room_floor | 199 | 0.7889 | 0.8595 | 0.8598 | 0.8819 | +0.0709 (+0.0634..+0.0784) |
| skycam | 281 | 0.8991 | 0.8991 | 0.8996 | 0.9003 | +0.0005 (-0.0021..+0.0036), ns |
| unknown | 401 | 0.9427 | 0.9695 | 0.9699 | 0.9668 | +0.0272 (+0.0234..+0.0311) |
| zed2023 | 645 | 0.9471 | 0.9502 | 0.9505 | 0.9489 | +0.0034 (+0.0029..+0.0038) |

`unknown` is 401 val frames that `field_index.json` does not list.

### Replica check against the C++ pipeline

The arms are Python reimplementations (`auto_battlebot/segmentation/preprocess.py`), so each was
checked against the real C++ pipeline. I replayed `data/svo/MassD_2026-08-29/2026-08-29T11-17-26.svo2`
through `build/auto_battlebot` with the x86 TensorRT engine
(`field_deeplabv3p_r50_2026-07-29_x86_64_sm89.engine`), recorded `/field_mask` and `/camera/image`,
recovered the label map from the overlay, and compared it with each arm run on the same frame.

| C++ build | Closest arm | IoU with it | Next closest |
| --- | --- | --- | --- |
| Before the fix | `bgr, imagenet, deployed` | 0.9967 | `bgr, dataset, deployed` 0.9925 |
| After the fix | `rgb, dataset, deployed` | 0.9895 | `rgb, imagenet, deployed` 0.9893 |

The remaining 0.3 to 1% is the PyTorch checkpoint against the TensorRT engine, plus the JPEG copy of
the frame the comparison reads.

### Recorded Jetson masks

On the 39 field-init frames in the MassD 2026-08-29 Jetson recordings, the recorded masks match the
BGR arms (IoU 0.880) better than the RGB arms (0.822 to 0.827), consistent with the robot running
BGR. The match is looser than on x86: the Jetson ran the aarch64 engine on the raw frame, and the
comparison runs PyTorch on the recorded JPEG.

| Arm | Floor fraction, 39 field inits | Floor fraction, 372 sampled frames | IoU with the training arm, sampled (mean, p10) |
| --- | --- | --- | --- |
| `bgr, imagenet, deployed` (before) | 0.2953 | 0.2869 | 0.853, 0.738 |
| `rgb, dataset, deployed` (after) | 0.2797 | 0.2843 | 0.889, 0.838 |

### Field fit on replays

`PointCloudFieldFilter` output after the first field init, six MassD SVOs from frame 0, before
and after builds on x86. The arena floor measures 2.26 m on a side
(`docs/reference/tools/synthgen_cage_specs.md`).

| SVO | Before, w x h (m) | After, w x h (m) | Largest contour, before / after (px) |
| --- | --- | --- | --- |
| 08-48-33 | 2.750 x 2.504 | 2.025 x 2.168 | 93,029 / 78,017 |
| 09-05-14 | 2.346 x 2.374 | 2.281 x 1.876 | 105,164 / 90,635 |
| 09-36-31 | 2.439 x 2.440 | 2.240 x 2.113 | 315,938 / 277,818 |
| 09-50-47 | 2.051 x 1.884 | 2.171 x 1.851 | 646,239 / 657,908 |
| 11-17-26 | 2.072 x 2.195 | 2.031 x 1.962 | 91,076 / 84,100 |
| 13-20-08 | 2.364 x 2.407 | 2.201 x 2.153 | 283,312 / 264,713 |
| Mean side, mean absolute error | 2.319 m, 0.199 m | 2.089 m, 0.174 m | |

The contour shrinks in 5 of 6, matching the smaller floor fraction above.

## Limitations

- One checkpoint. A model trained on BGR by accident would show the opposite; this one was trained on
  RGB (`semantic_train.py` converts with `cv2.COLOR_BGR2RGB`).
- The Jetson's own engine was not run. The aarch64 engine against the x86 one is unchecked here.
- The field-fit comparison is 6 inits from 6 SVOs, with no ground-truth field pose.

## Next

1. Deploy to the Jetson and confirm the recorded `/field_mask` matches the `rgb, dataset, deployed`
   arm the way the x86 replay does.
2. Score field-fit accuracy over many inits per recording (vary `svo_start_frame`) before reading
   anything into the 2.09 m mean side.
