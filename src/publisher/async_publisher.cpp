#include "publisher/async_publisher.hpp"

#include <chrono>
#include <utility>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {

AsyncPublisher::AsyncPublisher(std::shared_ptr<PublisherInterface> inner)
    : inner_(std::move(inner)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("async_publisher")),
      thread_([this] { run(); }) {}

AsyncPublisher::~AsyncPublisher() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    work_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void AsyncPublisher::enqueue(std::function<void()> call) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (pending_.size() >= kMaxPendingCalls) {
        const auto wait_start = std::chrono::steady_clock::now();
        space_cv_.wait(lock, [this] { return pending_.size() < kMaxPendingCalls; });
        const double blocked_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wait_start)
                .count();
        diagnostics_logger_->debug({{"blocked_ms", blocked_ms}});
    }
    pending_.push_back(std::move(call));
    lock.unlock();
    work_cv_.notify_one();
}

void AsyncPublisher::flush() {
    std::unique_lock<std::mutex> lock(mutex_);
    idle_cv_.wait(lock, [this] { return pending_.empty() && !busy_; });
}

void AsyncPublisher::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        work_cv_.wait(lock, [this] { return stop_ || !pending_.empty(); });
        // Stop only once the queue is empty, so the destructor drains it.
        if (pending_.empty()) return;
        std::function<void()> call = std::move(pending_.front());
        pending_.pop_front();
        busy_ = true;
        lock.unlock();
        space_cv_.notify_one();

        call();

        lock.lock();
        busy_ = false;
        if (pending_.empty()) idle_cv_.notify_all();
    }
}

void AsyncPublisher::publish_camera_data(const CameraData &data) {
    enqueue([this, data] { inner_->publish_camera_data(data); });
}

void AsyncPublisher::publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                                        const CameraInfo &camera_info) {
    enqueue([this, field_mask, image, camera_info] {
        inner_->publish_field_mask(field_mask, image, camera_info);
    });
}

void AsyncPublisher::publish_initial_field_description(
    const FieldDescriptionWithInlierPoints &field) {
    enqueue([this, field] { inner_->publish_initial_field_description(field); });
}

void AsyncPublisher::publish_field_description(
    const FieldDescription &field_description,
    const FieldDescriptionWithInlierPoints &initial_field_description) {
    // The inlier cloud is held by shared pointer, so this per-tick copy does not copy points.
    enqueue([this, field_description, initial_field_description] {
        inner_->publish_field_description(field_description, initial_field_description);
    });
}

void AsyncPublisher::publish_hazards(const FieldDescription &field_description) {
    enqueue([this, field_description] { inner_->publish_hazards(field_description); });
}

void AsyncPublisher::publish_robots(const RobotDescriptionsStamped &robots) {
    enqueue([this, robots] { inner_->publish_robots(robots); });
}

void AsyncPublisher::publish_blob_detections(const DetectionsStamped &detections) {
    enqueue([this, detections] { inner_->publish_blob_detections(detections); });
}

void AsyncPublisher::publish_keypoint_detections(const DetectionsStamped &detections) {
    enqueue([this, detections] { inner_->publish_keypoint_detections(detections); });
}

void AsyncPublisher::publish_navigation(const NavigationVisualization &nav) {
    enqueue([this, nav] { inner_->publish_navigation(nav); });
}

}  // namespace auto_battlebot
