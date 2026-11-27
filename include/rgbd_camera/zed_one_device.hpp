#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <sl/CameraOne.hpp>

#define AB_ZED_HAS_ANTIBANDING \
    (ZED_SDK_MAJOR_VERSION > 5 || (ZED_SDK_MAJOR_VERSION == 5 && ZED_SDK_MINOR_VERSION >= 5))

namespace auto_battlebot {

class ZedOneDevice {
   public:
    ZedOneDevice() = default;
    virtual ~ZedOneDevice() = default;
    ZedOneDevice(const ZedOneDevice &) = delete;
    ZedOneDevice &operator=(const ZedOneDevice &) = delete;

    virtual sl::ERROR_CODE open(const sl::InitParametersOne &params) { return zed_.open(params); }
    virtual void close() { zed_.close(); }
    virtual sl::ERROR_CODE grab() { return zed_.grab(); }
    virtual uint64_t image_timestamp_ns() {
        return zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    }
    virtual std::optional<cv::Mat> retrieve_bgra() {
        if (zed_.retrieveImage(zed_bgra_, sl::VIEW::LEFT, sl::MEM::CPU) !=
            sl::ERROR_CODE::SUCCESS) {
            return std::nullopt;
        }
        return cv::Mat(static_cast<int>(zed_bgra_.getHeight()),
                       static_cast<int>(zed_bgra_.getWidth()), CV_8UC4,
                       zed_bgra_.getPtr<sl::uchar1>(sl::MEM::CPU), zed_bgra_.getStepBytes());
    }
    virtual sl::CameraOneInformation information() { return zed_.getCameraInformation(); }
    virtual bool is_setting_supported(sl::VIDEO_SETTINGS setting) {
        return zed_.isCameraSettingSupported(setting);
    }
    virtual sl::ERROR_CODE get_setting(sl::VIDEO_SETTINGS setting, int &value) {
        return zed_.getCameraSettings(setting, value);
    }
    virtual sl::ERROR_CODE get_range(sl::VIDEO_SETTINGS setting, int &min, int &max) {
        return zed_.getCameraSettingsRange(setting, min, max);
    }
    virtual sl::ERROR_CODE set_setting(sl::VIDEO_SETTINGS setting, int value) {
        return zed_.setCameraSettings(setting, value);
    }

   private:
    sl::CameraOne zed_;
    sl::Mat zed_bgra_;
};

}  // namespace auto_battlebot
