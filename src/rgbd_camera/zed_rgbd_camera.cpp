#include "rgbd_camera/zed_rgbd_camera.hpp"

namespace auto_battlebot
{
    ZedRgbdCamera::ZedRgbdCamera(ZedRgbdCameraConfiguration &config) : zed_(sl::Camera()),
                                                                       is_initialized_(false),
                                                                       prev_tracking_state_(sl::POSITIONAL_TRACKING_STATE::LAST),
                                                                       position_tracking_enabled_(config.position_tracking)
    {
        params_ = sl::InitParameters();
        params_.camera_fps = config.camera_fps;
        params_.camera_resolution = get_zed_resolution(config.camera_resolution);
        params_.depth_mode = get_zed_depth_mode(config.depth_mode);
        params_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
        params_.coordinate_units = sl::UNIT::METER;

        // Set SVO file path if provided (for playback instead of live camera)
        if (!config.svo_file_path.empty())
        {
            params_.input.setFromSVOFile(config.svo_file_path.c_str());
            params_.svo_real_time_mode = config.svo_real_time_mode;
        }
    }

    ZedRgbdCamera::~ZedRgbdCamera()
    {
        if (is_initialized_)
        {
            zed_.close();
        }
    }

    bool ZedRgbdCamera::initialize()
    {
        sl::ERROR_CODE returned_state = zed_.open(params_);
        if (returned_state != sl::ERROR_CODE::SUCCESS)
        {
            std::cerr << "Failed to open ZED camera: " << sl::toString(returned_state) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            return false;
        }

        if (position_tracking_enabled_)
        {
            // Enable positional tracking for getting camera pose
            sl::PositionalTrackingParameters tracking_params;
            tracking_params.enable_imu_fusion = true;
            returned_state = zed_.enablePositionalTracking(tracking_params);
            if (returned_state != sl::ERROR_CODE::SUCCESS)
            {
                std::cerr << "Failed to enable positional tracking: " << sl::toString(returned_state) << std::endl;
                zed_.close();
                return false;
            }
        }
        else
        {
            std::cout << "Position tracking is disabled" << std::endl;
        }

        // Get camera information for intrinsics
        sl::CalibrationParameters calibration = zed_.getCameraInformation().camera_configuration.calibration_parameters;
        sl::Resolution image_size = zed_.getCameraInformation().camera_configuration.resolution;

        // Initialize camera info
        latest_data_.camera_info.width = static_cast<int>(image_size.width);
        latest_data_.camera_info.height = static_cast<int>(image_size.height);

        // Set intrinsics matrix (fx, 0, cx; 0, fy, cy; 0, 0, 1)
        latest_data_.camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        latest_data_.camera_info.intrinsics.at<double>(0, 0) = calibration.left_cam.fx;
        latest_data_.camera_info.intrinsics.at<double>(1, 1) = calibration.left_cam.fy;
        latest_data_.camera_info.intrinsics.at<double>(0, 2) = calibration.left_cam.cx;
        latest_data_.camera_info.intrinsics.at<double>(1, 2) = calibration.left_cam.cy;

        // Set distortion coefficients
        latest_data_.camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);
        latest_data_.camera_info.distortion.at<double>(0, 0) = calibration.left_cam.disto[0]; // k1
        latest_data_.camera_info.distortion.at<double>(0, 1) = calibration.left_cam.disto[1]; // k2
        latest_data_.camera_info.distortion.at<double>(0, 2) = calibration.left_cam.disto[2]; // p1
        latest_data_.camera_info.distortion.at<double>(0, 3) = calibration.left_cam.disto[3]; // p2
        latest_data_.camera_info.distortion.at<double>(0, 4) = calibration.left_cam.disto[4]; // k3

        is_initialized_ = true;
        return true;
    }

    bool ZedRgbdCamera::update()
    {
        if (!is_initialized_)
        {
            return false;
        }

        // Grab new frame
        sl::ERROR_CODE grab_status = zed_.grab();
        if (grab_status != sl::ERROR_CODE::SUCCESS)
        {
            if (grab_status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
            {
                should_close_ = true;
                std::cout << "End of SVO file reached." << std::endl;
            }
            else
            {
                std::cerr << "Failed to grab frame: " << sl::toString(grab_status) << std::endl;
            }
            return false;
        }

        // Retrieve RGB image
        sl::ERROR_CODE retrieve_status = zed_.retrieveImage(zed_rgb_, sl::VIEW::LEFT);
        if (retrieve_status != sl::ERROR_CODE::SUCCESS)
        {
            std::cerr << "Failed to retrieve RGB image: " << sl::toString(retrieve_status) << std::endl;
            return false;
        }

        // Retrieve depth map
        retrieve_status = zed_.retrieveMeasure(zed_depth_, sl::MEASURE::DEPTH);
        if (retrieve_status != sl::ERROR_CODE::SUCCESS)
        {
            std::cerr << "Failed to retrieve depth image: " << sl::toString(retrieve_status) << std::endl;
            return false;
        }

        // Get timestamp
        sl::Timestamp timestamp = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
        double stamp = static_cast<double>(timestamp.getNanoseconds()) / 1e9;
        latest_data_.tf_visodom_from_camera.header.stamp = stamp;
        latest_data_.tf_visodom_from_camera.header.frame_id = FrameId::VISUAL_ODOMETRY;
        latest_data_.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;

        // Get camera pose
        if (position_tracking_enabled_)
        {
            sl::POSITIONAL_TRACKING_STATE tracking_state = zed_.getPosition(zed_pose_, sl::REFERENCE_FRAME::WORLD);
            if (tracking_state != prev_tracking_state_)
            {
                std::cout << "Tracking state: " << tracking_state << std::endl;
                prev_tracking_state_ = tracking_state;
            }

            // Convert pose to transform matrix (4x4 Eigen matrix)
            sl::Transform zed_transform = zed_pose_.pose_data;
            latest_data_.tf_visodom_from_camera.transform.tf = Eigen::MatrixXd(4, 4);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    latest_data_.tf_visodom_from_camera.transform.tf(i, j) = zed_transform(i, j);
                }
            }
        }
        else
        {
            latest_data_.tf_visodom_from_camera.transform.tf = Eigen::MatrixXd::Identity(4, 4);
        }

        // Convert ZED RGB image to OpenCV Mat (BGRA to BGR)
        cv::Mat zed_rgb_mat(zed_rgb_.getHeight(), zed_rgb_.getWidth(), CV_8UC4, zed_rgb_.getPtr<sl::uchar1>());
        cv::cvtColor(zed_rgb_mat, latest_data_.rgb.image, cv::COLOR_BGRA2BGR);

        // Convert ZED depth image to OpenCV Mat (float32)
        cv::Mat zed_depth_mat(zed_depth_.getHeight(), zed_depth_.getWidth(), CV_32FC1, zed_depth_.getPtr<sl::uchar1>());
        zed_depth_mat.copyTo(latest_data_.depth.image);

        Header header;
        header.stamp = stamp;
        header.frame_id = FrameId::CAMERA;

        latest_data_.rgb.header = header;
        latest_data_.depth.header = header;
        latest_data_.camera_info.header = header;

        return true;
    }

    const CameraData &ZedRgbdCamera::get() const
    {
        return latest_data_;
    }

    bool ZedRgbdCamera::should_close()
    {
        return should_close_;
    }
} // namespace auto_battlebot
