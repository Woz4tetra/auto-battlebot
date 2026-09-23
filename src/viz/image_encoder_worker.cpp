#include "viz/image_encoder_worker.hpp"

#include <chrono>

namespace auto_battlebot {

ImageEncoderWorker::ImageEncoderWorker(std::string name, Process process)
    : name_(std::move(name)), process_(std::move(process)) {}

ImageEncoderWorker::~ImageEncoderWorker() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void ImageEncoderWorker::submit(Job job) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_) return;
        if (pending_) ++stats_.replaced_frames;
        pending_ = std::move(job);
        if (!thread_.joinable()) thread_ = std::thread([this] { run(); });
    }
    cv_.notify_one();
}

ImageEncoderWorker::Stats ImageEncoderWorker::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void ImageEncoderWorker::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        cv_.wait(lock, [this] { return stop_ || pending_.has_value(); });
        if (stop_) return;
        Job job = std::move(*pending_);
        pending_.reset();
        lock.unlock();

        const auto start = std::chrono::steady_clock::now();
        process_(job);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();

        lock.lock();
        ++stats_.encoded_frames;
        stats_.last_encode_ms = ms;
        total_encode_ms_ += ms;
        stats_.mean_encode_ms = total_encode_ms_ / static_cast<double>(stats_.encoded_frames);
    }
}

}  // namespace auto_battlebot
