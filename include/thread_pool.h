#pragma once

#include <vector>
#include <thread>
#include <functional>
#include <atomic>
#include "thread_safe_queue.h"

class ThreadPool {
public:
    using Task = std::function<void()>;

    explicit ThreadPool(size_t num_threads = std::thread::hardware_concurrency())
        : stop_(false) {
        workers_.reserve(num_threads);
        for (size_t i = 0; i < num_threads; ++i) {
            workers_.emplace_back([this] { worker_thread(); });
        }
    }

    ~ThreadPool() {
        shutdown();
    }

    // Submit a task to the pool
    void submit(Task task) {
        task_queue_.push(std::move(task));
    }

    // Get number of threads
    size_t thread_count() const {
        return workers_.size();
    }

    // Shutdown the pool
    void shutdown() {
        if (!stop_.exchange(true)) {
            task_queue_.shutdown();
            for (auto& worker : workers_) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
        }
    }

    // Wait for all tasks to complete
    void wait_for_tasks() {
        while (!task_queue_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    void worker_thread() {
        while (!stop_) {
            auto task = task_queue_.pop();
            if (task.has_value()) {
                (*task)();
            }
        }
    }

    std::vector<std::thread> workers_;
    ThreadSafeQueue<Task> task_queue_;
    std::atomic<bool> stop_;
};
