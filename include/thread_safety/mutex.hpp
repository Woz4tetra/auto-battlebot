#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

#if defined(__clang__)
#define AB_THREAD_ANNOTATION(x) __attribute__((x))
#else
#define AB_THREAD_ANNOTATION(x)
#endif

#define CAPABILITY(x) AB_THREAD_ANNOTATION(capability(x))
#define SCOPED_CAPABILITY AB_THREAD_ANNOTATION(scoped_lockable)
#define GUARDED_BY(x) AB_THREAD_ANNOTATION(guarded_by(x))
#define REQUIRES(...) AB_THREAD_ANNOTATION(requires_capability(__VA_ARGS__))
#define ACQUIRE(...) AB_THREAD_ANNOTATION(acquire_capability(__VA_ARGS__))
#define RELEASE(...) AB_THREAD_ANNOTATION(release_capability(__VA_ARGS__))
#define EXCLUDES(...) AB_THREAD_ANNOTATION(locks_excluded(__VA_ARGS__))

namespace auto_battlebot {

class MutexLock;

class CAPABILITY("mutex") Mutex {
   public:
    void lock() ACQUIRE() { mutex_.lock(); }
    void unlock() RELEASE() { mutex_.unlock(); }

   private:
    friend class MutexLock;
    std::mutex mutex_;
};

class SCOPED_CAPABILITY MutexLock {
   public:
    explicit MutexLock(Mutex &mutex) ACQUIRE(mutex) : lock_(mutex.mutex_) {}
    ~MutexLock() RELEASE() {}

    MutexLock(const MutexLock &) = delete;
    MutexLock &operator=(const MutexLock &) = delete;

    void wait(std::condition_variable &cv) { cv.wait(lock_); }

    template <class Rep, class Period>
    std::cv_status wait_for(std::condition_variable &cv,
                            const std::chrono::duration<Rep, Period> &timeout) {
        return cv.wait_for(lock_, timeout);
    }

   private:
    std::unique_lock<std::mutex> lock_;
};

}  // namespace auto_battlebot
