#include "rgbd_camera/zed_device.hpp"

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

namespace auto_battlebot {
namespace {
constexpr double kOpenWaitWarnMs = 2000.0;
constexpr double kGrabWarnMs = 250.0;
constexpr auto kOpenHardTimeout = std::chrono::seconds(30);

double elapsed_ms(const std::chrono::steady_clock::time_point &start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
}

void warn_if_slow(const std::chrono::steady_clock::time_point &start, const char *label,
                  double warn_ms) {
    const double ms = elapsed_ms(start);
    if (ms > warn_ms) {
        spdlog::warn("validation: {} slow elapsed_ms={:.2f}", label, ms);
    }
}

// Deliberate static leak: a std::future whose underlying task wedged inside the ZED SDK cannot be
// safely destroyed, because its destructor blocks on that task. On hard timeout the stuck future
// is moved here so it outlives the rest of the program and shutdown can proceed.
std::vector<std::future<sl::ERROR_CODE>> *g_leaked_open_futures =
    new std::vector<std::future<sl::ERROR_CODE>>;
std::mutex *g_leaked_open_futures_mutex = new std::mutex;

void leak_open_future(std::future<sl::ERROR_CODE> &&future) {
    std::lock_guard<std::mutex> lock(*g_leaked_open_futures_mutex);
    g_leaked_open_futures->push_back(std::move(future));
}

bool is_transient_grab_error(sl::ERROR_CODE error_code) {
    switch (error_code) {
        case sl::ERROR_CODE::CORRUPTED_FRAME:
        case sl::ERROR_CODE::CAMERA_REBOOTING:
            return true;
        default:
            return false;
    }
}
}  // namespace

bool ZedDevice::await_or_leak_open(const char *context, const char *validation_label) {
    const auto wait_start = std::chrono::steady_clock::now();
    if (pending_open_.wait_for(kOpenHardTimeout) == std::future_status::timeout) {
        spdlog::critical(
            "{}: zed_.open() stuck after {}s; leaking the open future for soft shutdown "
            "(the ZED SDK call completes in the background and is reaped at process exit)",
            context, kOpenHardTimeout.count());
        spdlog::default_logger()->flush();
        leak_open_future(std::move(pending_open_));
        return false;
    }
    warn_if_slow(wait_start, validation_label, kOpenWaitWarnMs);
    return true;
}

ZedDevice::OpenResult ZedDevice::open(const std::atomic<bool> &cancel) {
    pending_open_ =
        std::async(std::launch::async, [this]() -> sl::ERROR_CODE { return zed_.open(params_); });

    while (pending_open_.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (cancel) {
            // zed_.open() cannot be interrupted, so wait for it to return before leaving: the
            // handle must never be used concurrently or destroyed mid-open.
            if (!await_or_leak_open("ZedDevice::open cancel", "pending_open_wait_cancel")) {
                return OpenResult::Leaked;
            }
            return OpenResult::Cancelled;
        }
    }

    const sl::ERROR_CODE returned_state = pending_open_.get();
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to open ZED camera: {}", sl::toString(returned_state).c_str());
        return OpenResult::Failed;
    }
    return OpenResult::Opened;
}

bool ZedDevice::enable_tracking() {
    if (!tracking_enabled_) {
        spdlog::info("Position tracking is disabled");
        return true;
    }
    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = true;
    const sl::ERROR_CODE returned_state = zed_.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to enable positional tracking: {}",
                      sl::toString(returned_state).c_str());
        zed_.close();
        return false;
    }
    return true;
}

CameraInfo ZedDevice::read_camera_info() const {
    const sl::CalibrationParameters calibration =
        zed_.getCameraInformation().camera_configuration.calibration_parameters;
    const sl::Resolution image_size = zed_.getCameraInformation().camera_configuration.resolution;

    CameraInfo info;
    info.width = static_cast<int>(image_size.width);
    info.height = static_cast<int>(image_size.height);

    info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    info.intrinsics.at<double>(0, 0) = calibration.left_cam.fx;
    info.intrinsics.at<double>(1, 1) = calibration.left_cam.fy;
    info.intrinsics.at<double>(0, 2) = calibration.left_cam.cx;
    info.intrinsics.at<double>(1, 2) = calibration.left_cam.cy;

    info.distortion = cv::Mat::zeros(1, 5, CV_64F);
    info.distortion.at<double>(0, 0) = calibration.left_cam.disto[0];  // k1
    info.distortion.at<double>(0, 1) = calibration.left_cam.disto[1];  // k2
    info.distortion.at<double>(0, 2) = calibration.left_cam.disto[2];  // p1
    info.distortion.at<double>(0, 3) = calibration.left_cam.disto[3];  // p2
    info.distortion.at<double>(0, 4) = calibration.left_cam.disto[4];  // k3
    return info;
}

bool ZedDevice::is_svo_input() const {
    return zed_.getCameraInformation().input_type == sl::INPUT_TYPE::SVO;
}

ZedDevice::GrabStatus ZedDevice::grab() {
    sl::RuntimeParameters rt_params;
    rt_params.enable_depth = true;
    const auto grab_start = std::chrono::steady_clock::now();
    const sl::ERROR_CODE status = zed_.grab(rt_params);
    warn_if_slow(grab_start, "zed_grab", kGrabWarnMs);

    if (status == sl::ERROR_CODE::SUCCESS) return GrabStatus::Ok;
    if (is_transient_grab_error(status)) return GrabStatus::TransientError;
    if (status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) return GrabStatus::EndOfFile;

    spdlog::error("Failed to grab frame: {}", sl::toString(status).c_str());
    return GrabStatus::Fatal;
}

bool ZedDevice::retrieve(CameraData &out) {
    sl::ERROR_CODE retrieve_status = zed_.retrieveImage(zed_rgb_, sl::VIEW::LEFT);
    if (retrieve_status != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to retrieve RGB image: {}", sl::toString(retrieve_status).c_str());
        return false;
    }

    retrieve_status = zed_.retrieveMeasure(zed_depth_, sl::MEASURE::DEPTH);
    if (retrieve_status != sl::ERROR_CODE::SUCCESS) {
        spdlog::error("Failed to retrieve depth image: {}", sl::toString(retrieve_status).c_str());
        return false;
    }

    // TIME_REFERENCE::IMAGE is the capture instant live, and the original recording time in
    // replay, which lets a ManualClock drive the pipeline off the frame stamp.
    const sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
    const double stamp = static_cast<double>(timestamp.getNanoseconds()) / 1e9;
    out.frame_identity.image_stamp_ns = timestamp.getNanoseconds();

    out.tf_visodom_from_camera.header.stamp = stamp;
    out.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
    out.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;

    if (tracking_enabled_) {
        const sl::POSITIONAL_TRACKING_STATE tracking_state =
            zed_.getPosition(zed_pose_, sl::REFERENCE_FRAME::WORLD);
        out.tracking_ok = (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK);
        if (tracking_state != prev_tracking_state_) {
            spdlog::info("Tracking state: {}", sl::toString(tracking_state).c_str());
            prev_tracking_state_ = tracking_state;
        }

        const sl::Transform zed_transform = zed_pose_.pose_data;
        out.tf_visodom_from_camera.transform.tf = Eigen::MatrixXd(4, 4);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                out.tf_visodom_from_camera.transform.tf(i, j) = zed_transform(i, j);
            }
        }
    } else {
        out.tf_visodom_from_camera.transform.tf = Eigen::MatrixXd::Identity(4, 4);
        out.tracking_ok = true;
    }

    // Convert into a fresh Mat rather than reusing out.rgb.image. Consumers are handed a shallow
    // cv::Mat that shares this buffer, and cv::Mat::create() reuses an existing allocation whenever
    // size and type match without consulting the reference count, so converting straight into the
    // destination would overwrite pixels a consumer is still reading one frame behind. A per-frame
    // buffer lets that reference keep its allocation alive. One allocation per frame and no extra
    // copy: the conversion writes the full image either way.
    cv::Mat zed_rgb_mat(zed_rgb_.getHeight(), zed_rgb_.getWidth(), CV_8UC4,
                        zed_rgb_.getPtr<sl::uchar1>());
    cv::Mat rgb_frame;
    cv::cvtColor(zed_rgb_mat, rgb_frame, cv::COLOR_BGRA2BGR);
    out.rgb.image = rgb_frame;

    cv::Mat zed_depth_mat(zed_depth_.getHeight(), zed_depth_.getWidth(), CV_32FC1,
                          zed_depth_.getPtr<sl::uchar1>());
    cv::Mat depth_frame;
    zed_depth_mat.copyTo(depth_frame);
    out.depth.image = depth_frame;

    Header header;
    header.stamp = stamp;
    header.frame_id = FrameId::CAMERA;
    out.rgb.header = header;
    out.depth.header = header;
    out.camera_info.header = header;
    return true;
}

}  // namespace auto_battlebot
