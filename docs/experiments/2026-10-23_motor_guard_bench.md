# 2026-10-23 motor guard bench

## Notes moved from code comments

### General

- `mr_stabs_mk2/lib/motor_guard/motor_guard.cpp`: 2b: the shunt sets the total through one shared gain. Gated so idle and regen, where the sum is near zero or negative, do not move it. Also gated on load: free running, the model current is the small difference of d V and Ke w, and friction, eRPM lag, and duty error swamp it. On the 2026-10-23 bench run that pinned g at its 1.5 clamp.
- `mr_stabs_mk2/lib/motor_guard/motor_guard.cpp`: And on steady speed: the INA238 averages over 8.4 ms and eRPM lags a loop, so during a launch or reversal the shunt and the model describe different instants. A launch on the 2026-10-23 bench run dragged g to its 0.8 floor. The same goes for the duty: the INA238 sample spans up to the last two updates, and eRPM still reads the old speed on the sample a step lands in.
- `mr_stabs_mk2/lib/motor_guard/motor_guard.cpp`: The measured bound (2e a) caps the model too. The model assumes the ESC applies the commanded duty, but BLHeli holds duty back while it restarts a motor at low RPM: after a full-stick reversal on the 2026-10-23 bench run the model read 18 A against 2-5 A of pack current, and the limiter cut to 67%. The bound only holds while no motor regenerates, since regen adds a negative term to the pack current.
- `mr_stabs_mk2/lib/motor_guard/motor_guard.h`: g only raises the model. Over-estimates (a high R, or BLHeli holding duty back while it restarts a motor) are capped by the shunt bound instead: a duty clamp drove g to 0.8 on the 2026-10-23 bench run, which would have under-read every later stall.
- `mr_stabs_mk2/lib/motor_guard/motor_guard.h`: estimates below this do not drive the limiter The limiter acts on |I_hat| low-passed at this time constant. Launches and full-stick reversals draw stall to plugging current (up to about 47 A) for about 30 ms while back-EMF builds and eRPM catches up a loop late. Acting on the raw estimate cut a full-stick launch to 76% on the 2026-10-23 bench run. At 0.1 s a dual stall at full stick still crosses I_peak within about 60 ms. The thermal model integrates the raw estimate either way.
