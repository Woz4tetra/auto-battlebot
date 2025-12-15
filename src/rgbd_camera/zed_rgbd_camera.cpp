#include "rgbd_camera/zed_rgbd_camera.hpp"

namespace auto_battlebot
{
    ZedRgbdCamera::ZedRgbdCamera(ZedRgbdCameraConfiguration &config) : zed_(sl::Camera())
    {
        params_ = sl::InitParameters();
        params_.camera_fps = config.camera_fps;
        params_.camera_resolution = get_zed_resolution(config.camera_resolution);
        params_.depth_mode = get_zed_depth_mode(config.depth_mode);
        params_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
        params_.coordinate_units = sl::UNIT::MILLIMETER;
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
        return true;
    }
    bool ZedRgbdCamera::update()
    {
        return true;
    }
    bool ZedRgbdCamera::get(CameraData &data)
    {
        data = CameraData{};
        return false;
    }

} // namespace auto_battlebot
