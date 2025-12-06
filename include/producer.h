#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include "thread_safe_queue.h"

template<typename T>
class Producer {
public:
    using ProduceFunc = std::function<T()>;

    Producer(std::shared_ptr<ThreadSafeQueue<T>> queue, ProduceFunc produce_func)
        : queue_(queue), produce_func_(produce_func), running_(false) {}

    ~Producer() {
        stop();
    }

    void start() {
        if (!running_.exchange(true)) {
            thread_ = std::thread([this] { run(); });
        }
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    bool is_running() const {
        return running_;
    }

private:
    void run() {
        while (running_) {
            try {
                T item = produce_func_();
                queue_->push(std::move(item));
            } catch (const std::exception& e) {
                // Handle production errors
                // Could add error callback here
                break;
            }
        }
    }

    std::shared_ptr<ThreadSafeQueue<T>> queue_;
    ProduceFunc produce_func_;
    std::atomic<bool> running_;
    std::thread thread_;
};
