# 2026-10-23 motor guard bench

Bench run results behind the `motor_guard` gates. The design is in `docs/reference/robot_link/motor_guard.md`.

- Free running, the load gate was needed: without it the shared gain `g` pinned at its 1.5 clamp.
- Without the steady-speed gate, a launch dragged `g` to its 0.8 floor.
- A duty clamp drove `g` to 0.8, which would have under-read every later stall. This is why `g` only raises the model.
- After a full-stick reversal, the model read 18 A against 2-5 A of pack current, because BLHeli holds duty back while it restarts a motor at low RPM. The limiter cut to 67%. The measured bound now caps the model.
- Acting on the raw current estimate cut a full-stick launch to 76%. The low-pass on the limiter input removes that.
