# 2026-05 nhrl dropout gap quantiles

## Notes moved from code comments (2026-11-15)

### `playground/rl/fit_provisional_ekf_error.py`

- Pooled dropout gap quantiles from the May 2026 NHRL fights, on the sensing model's quantile grid [0, .01, .05, .1, .25, .5, .75, .9, .95, .99, 1]. Entries under p50 are interpolated from the 40 ms minimum gap. (at `MEASURED_GAP_QUANTILES_S = (`)

