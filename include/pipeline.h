#pragma once

#include <memory>
#include <vector>
#include "thread_safe_queue.h"
#include "producer.h"
#include "consumer.h"

// Multi-producer, multi-consumer pipeline
template<typename T>
class Pipeline {
public:
    using ProduceFunc = typename Producer<T>::ProduceFunc;
    using ConsumeFunc = typename Consumer<T>::ConsumeFunc;

    explicit Pipeline(size_t queue_capacity_hint = 100) 
        : queue_(std::make_shared<ThreadSafeQueue<T>>()) {}

    void add_producer(ProduceFunc produce_func) {
        producers_.push_back(
            std::make_unique<Producer<T>>(queue_, produce_func)
        );
    }

    void add_consumer(ConsumeFunc consume_func) {
        consumers_.push_back(
            std::make_unique<Consumer<T>>(queue_, consume_func)
        );
    }

    void start() {
        for (auto& producer : producers_) {
            producer->start();
        }
        for (auto& consumer : consumers_) {
            consumer->start();
        }
    }

    void stop() {
        // Stop producers first
        for (auto& producer : producers_) {
            producer->stop();
        }
        
        // Signal queue shutdown to wake up waiting consumers
        queue_->shutdown();
        
        // Stop consumers
        for (auto& consumer : consumers_) {
            consumer->stop();
        }
    }

    size_t queue_size() const {
        return queue_->size();
    }

private:
    std::shared_ptr<ThreadSafeQueue<T>> queue_;
    std::vector<std::unique_ptr<Producer<T>>> producers_;
    std::vector<std::unique_ptr<Consumer<T>>> consumers_;
};
