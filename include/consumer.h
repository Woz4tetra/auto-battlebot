#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include "thread_safe_queue.h"

template<typename T>
class Consumer {
public:
    using ConsumeFunc = std::function<void(T)>;

    Consumer(std::shared_ptr<ThreadSafeQueue<T>> queue, ConsumeFunc consume_func)
        : queue_(queue), consume_func_(consume_func), running_(false) {}

    ~Consumer() {
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
        while (running_ || !queue_->empty()) {
            auto item = queue_->pop();
            if (item.has_value()) {
                try {
                    consume_func_(std::move(*item));
                } catch (const std::exception& e) {
                    // Handle consumption errors
                    // Could add error callback here
                }
            } else if (queue_->is_shutdown()) {
                break;
            }
        }
    }

    std::shared_ptr<ThreadSafeQueue<T>> queue_;
    ConsumeFunc consume_func_;
    std::atomic<bool> running_;
    std::thread thread_;
};
