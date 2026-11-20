# 2026-11-15 DeepLab hull blob threshold

Measurement behind the `--min-blob-image-fraction` default (0.01) in `training/deeplab/convex_hull_masks.py`. How the filter works is in `docs/reference/perception/deeplab_floor_mask_data.md`.

## Result

- On the field corpus, runner-up blobs that survive the relative test (`--min-blob-fraction`, default 0.10) start around 0.36% of the image.
- The median dropped blob covers 0.08% of the image.
- An absolute floor between the two separates real field halves from specks.
- An occluder across the middle of the frame can leave one side of the field at 9% of the other. The relative test alone deletes that side.
