#include "publisher/ros_publisher.hpp"

#include "colorize_labels.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "ros/ros_message_adapters/ros_camera_info.hpp"
#include "ros/ros_message_adapters/ros_detections.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"
#include "ros/ros_message_adapters/ros_marker.hpp"
#include "ros/ros_message_adapters/ros_tf2.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
RosPublisher::RosPublisher(TypedPublisher<sensor_msgs::CompressedImage> rgb_image_publisher,
                           TypedPublisher<sensor_msgs::CameraInfo> camera_info_publisher,
                           TypedPublisher<sensor_msgs::CompressedImage> field_mask_publisher,
                           TypedPublisher<sensor_msgs::CameraInfo> field_mask_camera_info_publisher,
                           TypedPublisher<tf2_msgs::TFMessage> tf_publisher,
                           TypedPublisher<tf2_msgs::TFMessage> static_tf_publisher,
                           TypedPublisher<visualization_msgs::MarkerArray> field_marker_publisher,
                           TypedPublisher<visualization_msgs::MarkerArray> robot_marker_publisher,
                           TypedPublisher<visualization_msgs::MarkerArray> nav_marker_publisher,
                           TypedPublisher<std_msgs::String> blob_detections_publisher,
                           TypedPublisher<std_msgs::String> keypoint_detections_publisher,
                           TypedPublisher<std_msgs::String> frame_meta_publisher,
                           std::shared_ptr<McapRecorder> mcap_recorder)
    : rgb_image_publisher_(std::move(rgb_image_publisher)),
      camera_info_publisher_(std::move(camera_info_publisher)),
      field_mask_publisher_(std::move(field_mask_publisher)),
      field_mask_camera_info_publisher_(std::move(field_mask_camera_info_publisher)),
      tf_publisher_(std::move(tf_publisher)),
      static_tf_publisher_(std::move(static_tf_publisher)),
      field_marker_publisher_(std::move(field_marker_publisher)),
      robot_marker_publisher_(std::move(robot_marker_publisher)),
      nav_marker_publisher_(std::move(nav_marker_publisher)),
      blob_detections_publisher_(std::move(blob_detections_publisher)),
      keypoint_detections_publisher_(std::move(keypoint_detections_publisher)),
      frame_meta_publisher_(std::move(frame_meta_publisher)),
      mcap_recorder_(std::move(mcap_recorder)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("ros_publisher")) {}

void RosPublisher::publish_camera_data(const CameraData &data) {
    FunctionTimer timer(diagnostics_logger_, "publish_camera_data");

    // Publish RGB image. Compression is ~10 ms on the Jetson, so skip it entirely unless
    // someone is subscribed or the mcap actually records the topic.
    if (rgb_image_publisher_) {
        const bool has_subscriber = rgb_image_publisher_.num_subscribers() > 0;
        const bool mcap_records = mcap_recorder_ && mcap_recorder_->records_topic("/camera/image");
        if (has_subscriber || mcap_records) {
            auto rgb_msg = ros_adapters::to_ros_image_compressed(data.rgb);
            rgb_image_publisher_.publish(rgb_msg);
            if (mcap_recorder_) mcap_recorder_->write("/camera/image", rgb_msg);
        }
    }

    // Publish camera info
    if (camera_info_publisher_) {
        auto camera_info_msg = ros_adapters::to_ros_camera_info(data.camera_info);
        camera_info_publisher_.publish(camera_info_msg);
        if (mcap_recorder_) mcap_recorder_->write("/camera/camera_info", camera_info_msg);
    }

    // Publish which camera frame everything above came from. Without this a recording cannot be
    // joined back to its SVO frames: the header stamps here and the ones the SVO recorder writes
    // differ by roughly half a frame, so timestamps only ever give a nearest-neighbour guess.
    if (frame_meta_publisher_) {
        auto frame_meta_msg = ros_adapters::to_ros_frame_identity(data.frame_identity);
        frame_meta_publisher_.publish(frame_meta_msg);
        if (mcap_recorder_) mcap_recorder_->write("/camera/frame_meta", frame_meta_msg);
    }

    // Publish transform
    if (tf_publisher_) {
        auto tf_msg =
            ros_adapters::to_ros_tf_message(invert_transform(data.tf_visodom_from_camera));
        tf_publisher_.publish(tf_msg);
        if (mcap_recorder_) mcap_recorder_->write("/tf", tf_msg);
    }
}

void RosPublisher::publish_field_mask(const MaskStamped &field_mask, const RgbImage &image,
                                      const CameraInfo &camera_info) {
    FunctionTimer timer(diagnostics_logger_, "publish_field_mask");

    if (!field_mask_publisher_) {
        return;
    }
    // Colorize the mask labels
    cv::Mat colorized_mask = colorize_labels(field_mask.mask.mask);

    // Overlay the mask on the RGB image with 50% transparency
    cv::Mat overlay;
    if (!image.image.empty() && image.image.size() == colorized_mask.size() &&
        image.image.type() == CV_8UC3) {
        cv::addWeighted(image.image, 0.5, colorized_mask, 0.5, 0.0, overlay);
    } else {
        // If sizes don't match or image is invalid, just use the colorized mask
        overlay = colorized_mask;
    }

    // The mask image is captured once at field init and stays on screen while the camera keeps
    // moving. Stamp it and its camera info in CAMERA_WORLD (the camera pose at field init, held
    // in the TF tree by publish_field_description) so image panels project 3D markers through
    // the frozen pose instead of the live CAMERA frame.
    RgbImage mask_as_image;
    mask_as_image.header = field_mask.header;
    mask_as_image.header.frame_id = FrameId::CAMERA_WORLD;
    mask_as_image.image = overlay;

    auto mask_msg = ros_adapters::to_ros_image_compressed(mask_as_image);
    field_mask_publisher_.publish(mask_msg);
    if (mcap_recorder_) mcap_recorder_->write("/field_mask", mask_msg);

    if (field_mask_camera_info_publisher_) {
        CameraInfo field_mask_camera_info = camera_info;
        field_mask_camera_info.header.stamp = field_mask.header.stamp;
        field_mask_camera_info.header.frame_id = FrameId::CAMERA_WORLD;
        auto info_msg = ros_adapters::to_ros_camera_info(field_mask_camera_info);
        field_mask_camera_info_publisher_.publish(info_msg);
        if (mcap_recorder_) mcap_recorder_->write("/field_mask/camera_info", info_msg);
    }
}

void RosPublisher::publish_initial_field_description(
    const FieldDescriptionWithInlierPoints &field_description) {
    FunctionTimer timer(diagnostics_logger_, "publish_initial_field_description");

    if (field_marker_publisher_) {
        visualization_msgs::MarkerArray markers;
        markers.markers = ros_adapters::to_ros_field_marker(field_description);
        field_marker_publisher_.publish(markers);
        if (mcap_recorder_) mcap_recorder_->write("/field_markers", markers);
    }

    if (static_tf_publisher_) {
        // Publish static transform for field
        TransformStamped field_tf;
        field_tf.header = field_description.header;
        field_tf.child_frame_id = field_description.child_frame_id;
        field_tf.transform = field_description.tf_camera_from_fieldcenter;

        auto tf_msg = ros_adapters::to_ros_tf_message(invert_transform(field_tf));
        static_tf_publisher_.publish(tf_msg);
        if (mcap_recorder_) mcap_recorder_->write("/tf_static", tf_msg);
    }
}

void RosPublisher::publish_field_description(
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

    if (tf_publisher_) {
        auto tf_msg = ros_adapters::to_ros_tf_message(tfstamped_cameraworld_from_camera);
        tf_publisher_.publish(tf_msg);
        if (mcap_recorder_) mcap_recorder_->write("/tf", tf_msg);
    }
}

void RosPublisher::publish_robots(const RobotDescriptionsStamped &robots) {
    FunctionTimer timer(diagnostics_logger_, "publish_robots");

    if (robot_marker_publisher_) {
        visualization_msgs::MarkerArray markers;
        markers.markers = ros_adapters::to_ros_robot_markers(robots);
        robot_marker_publisher_.publish(markers);
        if (mcap_recorder_) mcap_recorder_->write("/robot_markers", markers);
    }
}

void RosPublisher::publish_blob_detections(const DetectionsStamped &detections) {
    FunctionTimer timer(diagnostics_logger_, "publish_blob_detections");
    publish_detections_on(blob_detections_publisher_, "/blob_detections", detections);
}

void RosPublisher::publish_keypoint_detections(const DetectionsStamped &detections) {
    FunctionTimer timer(diagnostics_logger_, "publish_keypoint_detections");
    publish_detections_on(keypoint_detections_publisher_, "/keypoint_detections", detections);
}

void RosPublisher::publish_detections_on(const TypedPublisher<std_msgs::String> &publisher,
                                         const std::string &topic,
                                         const DetectionsStamped &detections) {
    // A default-constructed header means the model never populated detections
    // (e.g. noop model); skip so recordings don't fill with stamp-zero messages.
    if (detections.header.frame_id == FrameId::EMPTY) {
        return;
    }
    if (publisher) {
        std_msgs::String msg = ros_adapters::to_ros_detections(detections);
        publisher.publish(msg);
        if (mcap_recorder_) {
            // Record with the frame stamp so runs of the same SVO align frame-exactly.
            const auto stamp_ns = static_cast<uint64_t>(detections.header.stamp * 1e9);
            mcap_recorder_->write(topic, msg, stamp_ns);
        }
    }
}

void RosPublisher::publish_navigation(const NavigationVisualization &nav) {
    FunctionTimer timer(diagnostics_logger_, "publish_navigation");

    if (nav_marker_publisher_) {
        visualization_msgs::MarkerArray markers;
        markers.markers = ros_adapters::to_ros_navigation_markers(nav);
        nav_marker_publisher_.publish(markers);
        if (mcap_recorder_) mcap_recorder_->write("/nav_markers", markers);
    }
}

}  // namespace auto_battlebot
