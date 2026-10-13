#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace auto_battlebot {

/**
 * Runs posted jobs one at a time, in post order, on its own thread. For slow work the caller
 * must not wait on, where the order of the jobs still matters.
 */
class SerialWorker {
   public:
    SerialWorker() : thread_([this] { run(); }) {}
    ~SerialWorker() { stop(); }

    SerialWorker(const SerialWorker &) = delete;
    SerialWorker &operator=(const SerialWorker &) = delete;

    /** Dropped after stop(). */
    void post(std::function<void()> job) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) return;
            jobs_.push_back(std::move(job));
        }
        cv_.notify_one();
    }

    /** Runs every job already posted, then joins. Idempotent. */
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_one();
        std::lock_guard<std::mutex> join_lock(join_mutex_);
        if (thread_.joinable()) thread_.join();
    }

   private:
    void run() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            cv_.wait(lock, [this] { return stop_ || !jobs_.empty(); });
            if (jobs_.empty()) return;
            std::function<void()> job = std::move(jobs_.front());
            jobs_.pop_front();
            lock.unlock();
            job();
            // Destroyed before relocking, so whatever the job captured is not freed under the
            // lock post() takes.
            job = nullptr;
            lock.lock();
        }
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> jobs_;
    bool stop_ = false;
    std::mutex join_mutex_;
    std::thread thread_;
};

}  // namespace auto_battlebot
