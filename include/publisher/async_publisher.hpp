#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "publisher/publisher_interface.hpp"

namespace auto_battlebot {

/**
 * Runs another publisher's calls on one worker thread, in call order, so publishing stays off the
 * runner's tick. The inner publisher only ever sees the worker thread.
 *
 * Each call copies its arguments into the queue. Camera frames are shallow cv::Mat copies, which
 * is safe because every camera hands out a freshly allocated frame each grab.
 *
 * The queue is bounded, and a full queue blocks the caller instead of dropping calls, so a
 * recording never loses a message. The tick pays for publishing only when the worker has fallen
 * a full queue behind, and the wait is logged as `blocked_ms`.
 */
class AsyncPublisher : public PublisherInterface {
   public:
    /** A tick makes seven publish calls, so this holds about nine ticks. */
    static constexpr size_t kMaxPendingCalls = 64;

    explicit AsyncPublisher(std::shared_ptr<PublisherInterface> inner);
    /** Drains the queue before joining, so queued frames still reach the sinks. */
    ~AsyncPublisher() override;

    AsyncPublisher(const AsyncPublisher &) = delete;
    AsyncPublisher &operator=(const AsyncPublisher &) = delete;
    AsyncPublisher(AsyncPublisher &&) = delete;
    AsyncPublisher &operator=(AsyncPublisher &&) = delete;

    void publish_camera_data(const CameraData &data) override;
    void publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                            const CameraInfo &camera_info) override;
    void publish_initial_field_description(const FieldDescriptionWithInlierPoints &field) override;
    void publish_field_description(
        const FieldDescription &field_description,
        const FieldDescriptionWithInlierPoints &initial_field_description) override;
    void publish_hazards(const FieldDescription &field_description) override;
    void publish_robots(const RobotDescriptionsStamped &robots) override;
    void publish_blob_detections(const DetectionsStamped &detections) override;
    void publish_keypoint_detections(const DetectionsStamped &detections) override;
    void publish_navigation(const NavigationVisualization &nav) override;
    void flush() override;

   private:
    void enqueue(std::function<void()> call);
    void run();

    std::shared_ptr<PublisherInterface> inner_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable space_cv_;
    std::condition_variable idle_cv_;
    std::deque<std::function<void()>> pending_;
    bool busy_ = false;
    bool stop_ = false;
    std::thread thread_;
};

}  // namespace auto_battlebot
