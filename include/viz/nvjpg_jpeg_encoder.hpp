#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "viz/jpeg_encoder.hpp"
#include "viz/nvjpg_plugin.h"

namespace auto_battlebot {

/**
 * JPEG on the Jetson's NVJPG engine through NvJPEGEncoder from the Jetson Multimedia API.
 * Compiled only when CMake finds the Multimedia API headers and L4T's libnvjpeg
 * (AUTO_BATTLEBOT_HAVE_NVJPG). That libnvjpeg is NVIDIA's libjpeg-compatible library with the
 * NVJPG extensions, not the CUDA toolkit's GPU nvJPEG: the GPU belongs to the detectors.
 *
 * libnvjpeg exports the whole libjpeg API without symbol versions, so once it is in the global
 * symbol scope it also answers OpenCV's, libtiff's and gdk-pixbuf's jpeg_* calls with its own,
 * larger structs, and every cv::imencode(".jpg") and cv::imdecode crashes. NvJPEGEncoder and
 * libnvjpeg therefore live in a plugin, libauto_battlebot_nvjpg.so next to this library, loaded
 * with RTLD_LOCAL so its symbols stay out of the global scope and RTLD_DEEPBIND so its own
 * jpeg_* calls reach libnvjpeg rather than OpenCV's libjpeg.
 *
 * NVJPG takes YCbCr 4:2:0, so each frame is converted from BGR on the CPU first. Odd widths and
 * heights lose their last column or row, since I420 needs even ones.
 */
class NvjpgJpegEncoder : public JpegEncoderInterface {
   public:
    /** nullptr when the plugin does not load or the engine does not open. */
    static std::unique_ptr<NvjpgJpegEncoder> create();
    ~NvjpgJpegEncoder() override;

    bool encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) override;
    std::string_view name() const override { return "nvjpg"; }

   private:
    struct Plugin {
        AutoBattlebotNvjpgCreate create = nullptr;
        AutoBattlebotNvjpgDestroy destroy = nullptr;
        AutoBattlebotNvjpgEncode encode = nullptr;
    };
    /** Loads the plugin on first use and keeps it for the life of the process. */
    static const Plugin *plugin();

    NvjpgJpegEncoder(const Plugin &plugin, void *encoder) : plugin_(plugin), encoder_(encoder) {}

    const Plugin &plugin_;
    void *encoder_;
    cv::Mat ycrcb_;
    std::vector<cv::Mat> planes_;
    cv::Mat i420_;
};

}  // namespace auto_battlebot
