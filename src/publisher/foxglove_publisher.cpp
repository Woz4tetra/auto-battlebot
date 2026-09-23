#include "publisher/foxglove_publisher.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "colorize_labels.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "foxglove_adapters/camera_info.hpp"
#include "foxglove_adapters/common.hpp"
#include "foxglove_adapters/detections.hpp"
#include "foxglove_adapters/image.hpp"
#include "foxglove_adapters/json_schemas.hpp"
#include "foxglove_adapters/scene.hpp"
#include "foxglove_adapters/transform.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {

namespace {
using foxglove::schemas::CameraCalibration;
using foxglove::schemas::CompressedImage;
using foxglove::schemas::CompressedVideo;
using foxglove::schemas::FrameTransforms;
using foxglove::schemas::ImageAnnotations;
using foxglove::schemas::SceneUpdate;

template <typename Message>
VizSchema sdk_schema() {
    return VizSchema::from_sdk(Message::schema());
}

VizSchema json_schema(const char *name, const char *text) {
    return VizSchema::jsonschema(name, text);
}

constexpr bool kLatched = true;
constexpr bool kUnlatched = false;
/** OpenCV's default, which /camera/image used before it moved to the worker. */
constexpr int kFullImageJpegQuality = 95;
constexpr int kPreviewVideoFps = 30;

/** A worker that JPEG-encodes on its own thread with its own encoder and logs the result. */
std::unique_ptr<ImageEncoderWorker> make_jpeg_worker(std::string name, OutputChannel &channel,
                                                     int quality) {
    std::shared_ptr<JpegEncoderInterface> encoder;
    return std::make_unique<ImageEncoderWorker>(
        std::move(name), [&channel, encoder, quality](const ImageEncoderWorker::Job &job) mutable {
            // Created on the worker thread, which is the only thread that uses it.
            if (!encoder) encoder = make_jpeg_encoder();
            CompressedImage image;
            image.timestamp = foxglove_adapters::to_timestamp(job.header.stamp);
            image.frame_id = foxglove_adapters::frame_id_string(job.header.frame_id);
            image.format = "jpeg";
            if (!encoder->encode(job.bgr, quality, image.data)) return;
            channel.log_message(image, job.log_time_ns);
        });
}
}  // namespace

FoxglovePublisher::FoxglovePublisher(std::shared_ptr<VizSink> sink,
                                     std::shared_ptr<McapRecorder> mcap_recorder,
                                     const FoxglovePublisherConfiguration &config)
    : sink_(std::move(sink)),
      mcap_recorder_(std::move(mcap_recorder)),
      rgb_image_("/camera/image", "protobuf", sdk_schema<CompressedImage>(), kUnlatched, sink_,
                 mcap_recorder_),
      camera_info_("/camera/camera_info", "protobuf", sdk_schema<CameraCalibration>(), kUnlatched,
                   sink_, mcap_recorder_),
      frame_meta_(
          "/camera/frame_meta", "json",
          json_schema(foxglove_adapters::kFrameMetaSchemaName, foxglove_adapters::kFrameMetaSchema),
          kUnlatched, sink_, mcap_recorder_),
      tf_("/tf", "protobuf", sdk_schema<FrameTransforms>(), kUnlatched, sink_, mcap_recorder_),
      field_mask_("/field_mask", "protobuf", sdk_schema<CompressedImage>(), kLatched, sink_,
                  mcap_recorder_),
      field_mask_camera_info_("/field_mask/camera_info", "protobuf",
                              sdk_schema<CameraCalibration>(), kLatched, sink_, mcap_recorder_),
      field_markers_("/field_markers", "protobuf", sdk_schema<SceneUpdate>(), kLatched, sink_,
                     mcap_recorder_),
      field_points_("/field_points", "protobuf", sdk_schema<foxglove::schemas::PointCloud>(),
                    kLatched, sink_, mcap_recorder_),
      hazard_markers_("/hazard_markers", "protobuf", sdk_schema<SceneUpdate>(), kLatched, sink_,
                      mcap_recorder_),
      robot_markers_("/robot_markers", "protobuf", sdk_schema<SceneUpdate>(), kLatched, sink_,
                     mcap_recorder_),
      nav_markers_("/nav_markers", "protobuf", sdk_schema<SceneUpdate>(), kLatched, sink_,
                   mcap_recorder_),
      blob_detections_("/blob_detections", "json",
                       json_schema(foxglove_adapters::kDetectionsSchemaName,
                                   foxglove_adapters::kDetectionsSchema),
                       kUnlatched, sink_, mcap_recorder_),
      // Annotations are a rendering of the JSON channel and are live only: the recorder is
      // handed nullptr so they can never land in a file.
      blob_annotations_("/blob_detections/annotations", "protobuf", sdk_schema<ImageAnnotations>(),
                        kUnlatched, sink_, nullptr),
      keypoint_detections_("/keypoint_detections", "json",
                           json_schema(foxglove_adapters::kDetectionsSchemaName,
                                       foxglove_adapters::kDetectionsSchema),
                           kUnlatched, sink_, mcap_recorder_),
      keypoint_annotations_("/keypoint_detections/annotations", "protobuf",
                            sdk_schema<ImageAnnotations>(), kUnlatched, sink_, nullptr),
      preview_image_("/camera/preview", "protobuf", sdk_schema<CompressedImage>(), kUnlatched,
                     sink_, nullptr),
      config_(config),
      field_mask_encoder_(make_jpeg_encoder()),
      diagnostics_logger_(DiagnosticsLogger::get_logger("foxglove_publisher")),
      image_worker_(make_jpeg_worker("camera_image", rgb_image_, kFullImageJpegQuality)),
      preview_worker_(
          make_jpeg_worker("camera_preview", preview_image_, config_.preview_jpeg_quality)) {
    if (sink_ && config_.preview_width > 0 && VideoEncoder::hardware_encoder_available()) {
        preview_video_ = std::make_unique<OutputChannel>("/camera/preview_video", "protobuf",
                                                         sdk_schema<CompressedVideo>(), kUnlatched,
                                                         sink_, nullptr);
    }
}

cv::Mat FoxglovePublisher::resize_for_preview(const cv::Mat &image) const {
    // Even dimensions: the H.264 encoder needs them, and the JPEG preview shares the frame.
    const int width = std::min(config_.preview_width, image.cols) & ~1;
    const int height = static_cast<int>(std::lround(static_cast<double>(image.rows) * width /
                                                    static_cast<double>(image.cols))) &
                       ~1;
    cv::Mat out;
    if (width == image.cols && height == image.rows) return image.clone();
    cv::resize(image, out, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
    return out;
}

void FoxglovePublisher::submit_preview_video(const cv::Mat &frame, uint64_t log_time) {
    if (!preview_video_encoder_.running()) {
        VideoEncoderOptions options;
        options.width = frame.cols;
        options.height = frame.rows;
        options.fps = kPreviewVideoFps;
        options.bitrate = static_cast<int64_t>(config_.preview_video_bitrate_kbps) * 1000;
        options.keyframe_interval = kPreviewVideoFps;
        options.input_is_uyvy = false;
        OutputChannel *channel = preview_video_.get();
        const bool started = preview_video_encoder_.start(
            options, [channel](const std::byte *data, size_t len, uint64_t log_time_ns, bool) {
                CompressedVideo message;
                message.timestamp =
                    foxglove_adapters::to_timestamp(static_cast<double>(log_time_ns) * 1e-9);
                message.frame_id = foxglove_adapters::frame_id_string(FrameId::CAMERA);
                message.format = "h264";
                message.data.assign(data, data + len);
                channel->log_message(message, log_time_ns);
            });
        if (!started || preview_video_encoder_.codec_name() == "libx264") {
            spdlog::warn("Preview video off: no hardware H.264 encoder opened ({})",
                         preview_video_encoder_.codec_name());
            preview_video_encoder_.stop();
            preview_video_failed_ = true;
            return;
        }
    }
    preview_video_encoder_.submit(frame, log_time);
}

void FoxglovePublisher::submit_previews(const RgbImage &rgb, uint64_t log_time) {
    if (rgb.image.empty() || config_.preview_width <= 0) return;
    const auto now = std::chrono::steady_clock::now();
    const bool want_jpeg =
        preview_image_.has_consumers() &&
        (config_.preview_rate_hz <= 0.0 ||
         now - last_preview_ >= std::chrono::duration<double>(1.0 / config_.preview_rate_hz));

    bool want_video = false;
    if (preview_video_ && !preview_video_failed_) {
        const uint32_t subscribers = preview_video_->num_subscribers();
        // A viewer that just subscribed cannot decode until the next IDR; ask for one now.
        if (subscribers > preview_video_subscribers_) preview_video_encoder_.request_keyframe();
        preview_video_subscribers_ = subscribers;
        want_video = subscribers > 0 && now - last_preview_video_ >=
                                            std::chrono::duration<double>(1.0 / kPreviewVideoFps);
    }
    if (!want_jpeg && !want_video) return;

    cv::Mat small = resize_for_preview(rgb.image);
    if (want_video) {
        last_preview_video_ = now;
        submit_preview_video(small, log_time);
    }
    if (want_jpeg) {
        last_preview_ = now;
        preview_worker_->submit({std::move(small), rgb.header, log_time});
    }
}

void FoxglovePublisher::log_encoder_stats() {
    for (const auto *worker : {image_worker_.get(), preview_worker_.get()}) {
        const auto stats = worker->stats();
        if (stats.encoded_frames == 0) continue;
        diagnostics_logger_->debug(worker->name(),
                                   {{"encoded_frames", static_cast<int>(stats.encoded_frames)},
                                    {"replaced_frames", static_cast<int>(stats.replaced_frames)},
                                    {"encode_ms", stats.last_encode_ms},
                                    {"mean_encode_ms", stats.mean_encode_ms}});
    }
}

void FoxglovePublisher::publish_camera_data(const CameraData &data) {
    FunctionTimer timer(diagnostics_logger_, "publish_camera_data");
    const uint64_t log_time = wall_time_ns();

    // JPEG compression is ~10 ms of CPU on the Jetson, so it runs on worker threads, and only
    // when a client subscribed or the recorder records the topic. The loop pays one copy of the
    // frame, or one resize for the preview.
    if (rgb_image_.has_consumers() && !data.rgb.image.empty()) {
        image_worker_->submit({data.rgb.image.clone(), data.rgb.header, log_time});
    }
    submit_previews(data.rgb, log_time);
    log_encoder_stats();

    auto calibration = foxglove_adapters::to_camera_calibration(data.camera_info);
    camera_info_.log_message(calibration, log_time);

    frame_meta_.log(foxglove_adapters::to_frame_meta_json(data.frame_identity), log_time);

    auto tf = foxglove_adapters::to_frame_transforms(data.tf_visodom_from_camera);
    tf_.log_message(tf, log_time);

    if (sink_) {
        diagnostics_logger_->debug({{"viz_connected", sink_->connected() ? 1 : 0},
                                    {"viz_dropped", static_cast<int>(sink_->dropped_messages())}});
    }
}

void FoxglovePublisher::publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                                           const CameraInfo &camera_info) {
    FunctionTimer timer(diagnostics_logger_, "publish_field_mask");
    const uint64_t log_time = wall_time_ns();

    cv::Mat colorized_mask = colorize_labels(field_mask.mask.mask);
    cv::Mat overlay;
    if (!image.image.empty() && image.image.size() == colorized_mask.size() &&
        image.image.type() == CV_8UC3) {
        cv::addWeighted(image.image, 0.5, colorized_mask, 0.5, 0.0, overlay);
    } else {
        overlay = colorized_mask;
    }

    // The mask image is captured once at field init and stays on screen while the camera keeps
    // moving. Stamp it and its camera info in CAMERA_WORLD (the camera pose at field init, held
    // in the TF tree by publish_field_description) so image panels project 3D markers through
    // that frozen pose instead of the live CAMERA frame.
    RgbImage mask_as_image;
    mask_as_image.header = field_mask.header;
    mask_as_image.header.frame_id = FrameId::CAMERA_WORLD;
    mask_as_image.image = overlay;
    // Once per field fit, so it encodes inline, but on the same encoder the workers use.
    CompressedImage mask_message;
    mask_message.timestamp = foxglove_adapters::to_timestamp(mask_as_image.header.stamp);
    mask_message.frame_id = foxglove_adapters::frame_id_string(mask_as_image.header.frame_id);
    mask_message.format = "jpeg";
    field_mask_encoder_->encode(mask_as_image.image, kFullImageJpegQuality, mask_message.data);
    field_mask_.log_message(mask_message, log_time);

    CameraInfo field_mask_camera_info = camera_info;
    field_mask_camera_info.header.stamp = field_mask.header.stamp;
    field_mask_camera_info.header.frame_id = FrameId::CAMERA_WORLD;
    auto calibration = foxglove_adapters::to_camera_calibration(field_mask_camera_info);
    field_mask_camera_info_.log_message(calibration, log_time);
}

void FoxglovePublisher::publish_initial_field_description(
    const FieldDescriptionWithInlierPoints &field_description) {
    FunctionTimer timer(diagnostics_logger_, "publish_initial_field_description");
    const uint64_t log_time = wall_time_ns();

    auto border = foxglove_adapters::to_field_scene(field_description);
    field_markers_.log_message(border, log_time);

    if (auto cloud = foxglove_adapters::to_field_point_cloud(field_description)) {
        field_points_.log_message(*cloud, log_time);
    }
}

void FoxglovePublisher::publish_field_description(
    const FieldDescription &field_description,
    const FieldDescriptionWithInlierPoints &initial_field_description) {
    FunctionTimer timer(diagnostics_logger_, "publish_field_description");

    Eigen::MatrixXd tf_cameraworld_from_fieldcenter =
        initial_field_description.tf_camera_from_fieldcenter.tf;
    Eigen::MatrixXd tf_camera_from_fieldcenter = field_description.tf_camera_from_fieldcenter.tf;
    Eigen::MatrixXd tf_cameraworld_from_camera =
        tf_cameraworld_from_fieldcenter * tf_camera_from_fieldcenter.inverse();
    TransformStamped tfstamped_cameraworld_from_camera{};
    tfstamped_cameraworld_from_camera.header.stamp = field_description.header.stamp;
    tfstamped_cameraworld_from_camera.header.frame_id = initial_field_description.header.frame_id;
    tfstamped_cameraworld_from_camera.child_frame_id = field_description.header.frame_id;
    tfstamped_cameraworld_from_camera.transform.tf = tf_cameraworld_from_camera;

    // field -> camera_world does not change between field initializations, but it is republished
    // every cycle rather than latched once: a recording that starts (or resumes) after field init
    // would otherwise never contain the edge every camera->field lookup needs.
    TransformStamped tfstamped_cameraworld_from_fieldcenter{};
    tfstamped_cameraworld_from_fieldcenter.header.stamp = field_description.header.stamp;
    tfstamped_cameraworld_from_fieldcenter.header.frame_id =
        initial_field_description.header.frame_id;
    tfstamped_cameraworld_from_fieldcenter.child_frame_id =
        initial_field_description.child_frame_id;
    tfstamped_cameraworld_from_fieldcenter.transform.tf = tf_cameraworld_from_fieldcenter;

    foxglove::schemas::FrameTransforms tf_message;
    tf_message.transforms.push_back(foxglove_adapters::to_frame_transform(
        invert_transform(tfstamped_cameraworld_from_fieldcenter)));
    tf_message.transforms.push_back(
        foxglove_adapters::to_frame_transform(tfstamped_cameraworld_from_camera));
    tf_.log_message(tf_message, wall_time_ns());
}

void FoxglovePublisher::publish_hazards(const FieldDescription &field_description) {
    FunctionTimer timer(diagnostics_logger_, "publish_hazards");
    // Published even when there are no hazards: the update's delete-all clears last cycle's
    // rings, and a replay has to be able to say what the controller knew.
    auto scene = foxglove_adapters::to_hazard_scene(field_description);
    hazard_markers_.log_message(scene, wall_time_ns());
}

void FoxglovePublisher::publish_robots(const RobotDescriptionsStamped &robots) {
    FunctionTimer timer(diagnostics_logger_, "publish_robots");
    auto scene = foxglove_adapters::to_robot_scene(robots);
    robot_markers_.log_message(scene, wall_time_ns());
}

void FoxglovePublisher::publish_blob_detections(const DetectionsStamped &detections) {
    FunctionTimer timer(diagnostics_logger_, "publish_blob_detections");
    publish_detections_on(blob_detections_, blob_annotations_, detections);
}

void FoxglovePublisher::publish_keypoint_detections(const DetectionsStamped &detections) {
    FunctionTimer timer(diagnostics_logger_, "publish_keypoint_detections");
    publish_detections_on(keypoint_detections_, keypoint_annotations_, detections);
}

void FoxglovePublisher::publish_detections_on(OutputChannel &json_channel,
                                              OutputChannel &annotations_channel,
                                              const DetectionsStamped &detections) {
    // A default-constructed header means the model never populated detections
    // (e.g. noop model); skip so recordings don't fill with stamp-zero messages.
    if (detections.header.frame_id == FrameId::EMPTY) return;

    // Record with the frame stamp so runs of the same SVO align frame-exactly.
    const auto stamp_ns = static_cast<uint64_t>(detections.header.stamp * 1e9);
    json_channel.log(foxglove_adapters::to_detections_json(detections), stamp_ns);

    if (annotations_channel.num_subscribers() > 0) {
        auto annotations = foxglove_adapters::to_image_annotations(detections);
        annotations_channel.log_message(annotations, stamp_ns);
    }
}

void FoxglovePublisher::publish_navigation(const NavigationVisualization &nav) {
    FunctionTimer timer(diagnostics_logger_, "publish_navigation");
    auto scene = foxglove_adapters::to_navigation_scene(nav);
    nav_markers_.log_message(scene, wall_time_ns());
}

}  // namespace auto_battlebot
