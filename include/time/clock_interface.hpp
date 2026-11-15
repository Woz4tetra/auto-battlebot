#pragma once

namespace auto_battlebot {
/**
 * @brief Source of logical (control) time in seconds.
 *
 * Logical time is what the control loop uses for derivative terms, prediction horizons, and message
 * stamps. It is distinct from wall-clock time used for profiling, loop pacing, and hardware
 * timeouts, which must keep using std::chrono directly.
 *
 * The Runner sets the clock from the camera frame timestamp each tick (see ManualClock), so a
 * single source of time drives the whole pipeline and sim/playback runs become deterministic and
 * reproducible.
 */
class ClockInterface {
   public:
    virtual ~ClockInterface() = default;

    virtual double now() const = 0;

    virtual void set([[maybe_unused]] double seconds) {}
};

}  // namespace auto_battlebot
