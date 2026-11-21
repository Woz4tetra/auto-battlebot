# Thread-safety annotations

`include/thread_safety/mutex.hpp` has an annotated `Mutex`, a scoped `MutexLock`, and the Clang attribute macros. Under GCC the macros expand to nothing. `scripts/lint` passes `-Wthread-safety` to clang-tidy over `src/`, so lint fails on any unguarded access.

| Macro | Put it on |
| --- | --- |
| `GUARDED_BY(mu)` | A member that is only read or written with `mu` held |
| `REQUIRES(mu)` | A method the caller must call with `mu` held (the `_locked` helpers) |
| `EXCLUDES(mu)` | A method that takes `mu` itself |

Conventions:

- Lock with `MutexLock lock(mutex_);`. It holds a `std::unique_lock`, so `std::condition_variable` works through it.
- Wait in a loop in the caller's scope: `while (!ready) lock.wait(cv_);`. A predicate lambda is analysed as a separate function, so it cannot read guarded members.
- For a timed wait, `lock.wait_for(cv_, timeout)` returns `std::cv_status`; check the condition again after a timeout.
- Constructors and destructors are not analysed.

Annotated today: `McapRecorder`, `VideoEncoder`, `ZedRgbdCamera`.
