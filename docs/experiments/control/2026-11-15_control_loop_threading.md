# Control loop drivers

`ControlLoopInterface` drives `ControlLoop::run_cycle()`. The two implementations differ only in what schedules a cycle.

- `SteppedControlLoop` runs cycles synchronously from the perception tick, which keeps playback reproducible.
- `ThreadedControlLoop` runs them on its own thread at `rate_hz`, so the command tracks the filter's evolving estimate instead of freezing between perception frames.

Transmitter input: stepped drivers read it on the caller's thread, so a single-threaded run reads the transmitter before the camera grab. Threaded drivers own the transmitter on their own thread and ignore that call.
