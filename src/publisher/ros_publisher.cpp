#include "publisher/ros_publisher.hpp"

#include "colorize_labels.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "ros/ros_message_adapters/ros_camera_info.hpp"
#include "ros/ros_message_adapters/ros_image.hpp"
#include "ros/ros_message_adapters/ros_marker.hpp"
#include "ros/ros_message_adapters/ros_tf2.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
RosPublisher::RosPublisher(TypedPublisher<sensor_msgs::CompressedImage> rgb_image_publisher,
                           TypedPublisher<sensor_msgs::CameraInfo> camera_info_publisher,
                           TypedPublisher<sensor_msgs::CompressedImage> field_mask_publisher,
                           TypedPublisher<tf2_msgs::TFMessage> tf_publisher,
                           TypedPublisher<tf2_msgs::TFMessage> static_tf_publisher,
                           TypedPublisher<visualization_msgs::MarkerArray> field_marker_publisher,
                           TypedPublisher<visualization_msgs::MarkerArray> robot_marker_publisher,
                           TypedPublisher<visualization_msgs::MarkerArray> nav_marker_publisher,
                           std::shared_ptr<McapRecorder> mcap_recorder)
    : rgb_image_publisher_(std::move(rgb_image_publisher)),
      camera_info_publisher_(std::move(camera_info_publisher)),
      field_mask_publisher_(std::move(field_mask_publisher)),
      tf_publisher_(std::move(tf_publisher)),
      static_tf_publisher_(std::move(static_tf_publisher)),
      field_marker_publisher_(std::move(field_marker_publisher)),
      robot_marker_publisher_(std::move(robot_marker_publisher)),
      nav_marker_publisher_(std::move(nav_marker_publisher)),
      mcap_recorder_(std::move(mcap_recorder)),
      diagnostics_logger_(DiagnosticsLogger::get_logger("ros_publisher")) {}

void RosPublisher::publish_camera_data(const CameraData &data) {
    FunctionTimer timer(diagnostics_logger_, "publish_camera_data");

    // Publish RGB image
    if (rgb_image_publisher_) {
        auto rgb_msg = ros_adapters::to_ros_image_compressed(data.rgb);
        rgb_image_publisher_.publish(rgb_msg);
        if (mcap_recorder_) mcap_recorder_->write("/camera/image", rgb_msg);
    }

    // Publish camera info
    if (camera_info_publisher_) {
        auto camera_info_msg = ros_adapters::to_ros_camera_info(data.camera_info);
        camera_info_publisher_.publish(camera_info_msg);
        if (mcap_recorder_) mcap_recorder_->write("/camera/camera_info", camera_info_msg);
    }

    // Publish transform
    if (tf_publisher_) {
        auto tf_msg =
            ros_adapters::to_ros_tf_message(invert_transform(data.tf_visodom_from_camera));
        tf_publisher_.publish(tf_msg);
        if (mcap_recorder_) mcap_recorder_->write("/tf", tf_msg);
    }
}

void RosPublisher::publish_field_mask(const MaskStamped &field_mask, const RgbImage &image) {
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

    RgbImage mask_as_image;
    mask_as_image.header = field_mask.header;
    mask_as_image.image = overlay;

    auto mask_msg = ros_adapters::to_ros_image_compressed(mask_as_image);
    field_mask_publisher_.publish(mask_msg);
    if (mcap_recorder_) mcap_recorder_->write("/field_mask", mask_msg);
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
