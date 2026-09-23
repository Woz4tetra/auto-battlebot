#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <thread>

#include "data_structures/header.hpp"

namespace auto_battlebot {

/**
 * Runs image encoding off the perception loop. Modeled on VideoEncoder.
 *
 * submit() never blocks: the worker keeps one pending frame and a newer frame replaces it, since
 * a preview wants the newest frame, not every frame. The thread starts on the first submit, so a
 * topic nobody subscribes to never costs a thread.
 */
class ImageEncoderWorker {
   public:
    struct Job {
        cv::Mat bgr;
        Header header;
        uint64_t log_time_ns = 0;
    };
    /** Encodes and logs one frame on the worker thread. */
    using Process = std::function<void(const Job &job)>;

    struct Stats {
        uint64_t encoded_frames = 0;
        uint64_t replaced_frames = 0;
        double last_encode_ms = 0.0;
        double mean_encode_ms = 0.0;
    };

    ImageEncoderWorker(std::string name, Process process);
    ~ImageEncoderWorker();
    ImageEncoderWorker(const ImageEncoderWorker &) = delete;
    ImageEncoderWorker &operator=(const ImageEncoderWorker &) = delete;

    /** Takes ownership of the frame; the caller must not reuse its buffer (clone or resize into a
     *  new Mat first). */
    void submit(Job job);

    Stats stats() const;
    const std::string &name() const { return name_; }

   private:
    void run();

    std::string name_;
    Process process_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<Job> pending_;
    Stats stats_;
    double total_encode_ms_ = 0.0;
    bool stop_ = false;
    std::thread thread_;
};

}  // namespace auto_battlebot
