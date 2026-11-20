#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "viz/jpeg_encoder.hpp"
#include "viz/nvjpg_plugin.h"

namespace auto_battlebot {

class NvjpgJpegEncoder : public JpegEncoderInterface {
   public:
    static std::unique_ptr<NvjpgJpegEncoder> create();
    ~NvjpgJpegEncoder() override;

    std::string_view name() const override { return "nvjpg"; }

   protected:
    bool encode_on_owner_thread(const cv::Mat &bgr, int quality,
                                std::vector<std::byte> &out) override;

   private:
    struct Plugin {
        AutoBattlebotNvjpgCreate create = nullptr;
        AutoBattlebotNvjpgDestroy destroy = nullptr;
        AutoBattlebotNvjpgEncode encode = nullptr;
    };
    static const Plugin *plugin();

    NvjpgJpegEncoder(const Plugin &plugin, void *encoder) : plugin_(plugin), encoder_(encoder) {}

    const Plugin &plugin_;
    void *encoder_;
    cv::Mat ycrcb_;
    std::vector<cv::Mat> planes_;
    cv::Mat i420_;
};

}  // namespace auto_battlebot
