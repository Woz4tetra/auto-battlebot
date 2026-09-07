#include "publisher/foxglove_publisher.hpp"

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
}  // namespace

FoxglovePublisher::FoxglovePublisher(std::shared_ptr<VizSink> sink,
                                     std::shared_ptr<McapRecorder> mcap_recorder)
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
      diagnostics_logger_(DiagnosticsLogger::get_logger("foxglove_publisher")) {}

void FoxglovePublisher::publish_camera_data(const CameraData &data) {
    FunctionTimer timer(diagnostics_logger_, "publish_camera_data");
    const uint64_t log_time = wall_time_ns();

    // JPEG compression is ~10 ms on the Jetson, so skip it entirely unless a Foxglove client
    // subscribed or the recorder actually records the topic.
    if (rgb_image_.has_consumers()) {
        auto image = foxglove_adapters::to_compressed_image(data.rgb);
        rgb_image_.log_message(image, log_time);
    }

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
    auto mask_message = foxglove_adapters::to_compressed_image(mask_as_image);
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
