# 2026-11-15 free roam field mask

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/mask_model/free_roam_mask_model.cpp`

Beside `seed_rect = centered_rect(image.image.size(), seed_patch_fraction_);`:

Median, not mean: a robot or debris overlapping the seed patch must not skew the seed.

### `src/mask_model/free_roam_mask_model.cpp`

Beside `cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));`:

Close small speckle holes; the largest-contour pass downstream discards islands, so the mask only needs one solid field region.

### `tests/mask_model/test_free_roam_mask_model.cpp`

Beside `LightingRampStaysInMask`:

The test image is a horizontal brightness ramp on gray material, 90 on the left to 160 on the right. That moves mostly L in Lab, which the loose `tolerance_l` must absorb.
