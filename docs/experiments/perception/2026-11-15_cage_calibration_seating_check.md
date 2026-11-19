# 2026-11-15 cage calibration seating check

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/field_filter/calibrated_field_filter.cpp`

Beside `const auto expected =`:

Compare where the stored calibration says the corners are against where the live fit puts them. Corners rather than a pose difference, because pixels are the units the failure shows up in and a few pixels of corner error is the threshold an operator can reason about.
