# Velocity-prediction A/B: results

- ON  ticks=2652 gaps=165 live=0.635
- OFF ticks=2582 gaps=203 live=0.684

| metric | ON | OFF |
| --- | --- | --- |
| n_gaps | 165.0000 | 203.0000 |
| mean_gap_ms | 150.0025 | 112.7255 |
| mean_resume_error_m | 0.2190 | 0.2660 |
| median_resume_error_m | 0.1386 | 0.1456 |
| mean_frozen_ref_error_m | 0.2680 | 0.2801 |
| mean_tick_jump_m | 0.0621 | 0.0568 |

Mean resume error is 0.0470 m lower with prediction ON (0.2190 vs 0.2660 m). Positive means prediction reduced the post-dropout position error.

Because prediction also changes data association (it advances last_position), the two arms do not have identical gap sets, so the table below bins by gap length to compare at matched dropout durations.

| gap length (ms) | n ON | n OFF | resume err ON (m) | resume err OFF (m) |
| --- | --- | --- | --- | --- |
| 40–80 | 84 | 107 | 0.1095 | 0.1569 |
| 80–120 | 26 | 41 | 0.1406 | 0.2927 |
| 120–200 | 20 | 31 | 0.3188 | 0.5452 |
| 200–400 | 22 | 16 | 0.3921 | 0.3712 |
| 400–2000 | 13 | 8 | 0.6373 | 0.2954 |
