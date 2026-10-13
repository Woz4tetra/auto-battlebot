#include "viz/nvjpg_jpeg_encoder.hpp"

#include <dlfcn.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>

namespace auto_battlebot {

const NvjpgJpegEncoder::Plugin *NvjpgJpegEncoder::plugin() {
    static const std::optional<Plugin> loaded = []() -> std::optional<Plugin> {
        // The plugin is built and installed next to this library.
        Dl_info info{};
        if (dladdr(reinterpret_cast<void *>(&NvjpgJpegEncoder::plugin), &info) == 0 ||
            info.dli_fname == nullptr) {
            spdlog::warn("NVJPG: could not find this library's path; using OpenCV");
            return std::nullopt;
        }
        const std::string path =
            (std::filesystem::path(info.dli_fname).parent_path() / AUTO_BATTLEBOT_NVJPG_PLUGIN)
                .string();
        void *handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
        if (handle == nullptr) {
            spdlog::warn("NVJPG plugin did not load ({}); using OpenCV", dlerror());
            return std::nullopt;
        }
        Plugin plugin;
        plugin.create = reinterpret_cast<AutoBattlebotNvjpgCreate>(
            dlsym(handle, "auto_battlebot_nvjpg_create"));
        plugin.destroy = reinterpret_cast<AutoBattlebotNvjpgDestroy>(
            dlsym(handle, "auto_battlebot_nvjpg_destroy"));
        plugin.encode = reinterpret_cast<AutoBattlebotNvjpgEncode>(
            dlsym(handle, "auto_battlebot_nvjpg_encode"));
        if (!plugin.create || !plugin.destroy || !plugin.encode) {
            spdlog::warn("NVJPG plugin {} is missing a function; using OpenCV", path);
            return std::nullopt;
        }
        return plugin;
    }();
    return loaded ? &*loaded : nullptr;
}

std::unique_ptr<NvjpgJpegEncoder> NvjpgJpegEncoder::create() {
    const Plugin *loaded = plugin();
    if (!loaded) return nullptr;
    void *encoder = loaded->create();
    if (!encoder) {
        spdlog::warn("NVJPG encoder did not open; using OpenCV");
        return nullptr;
    }
    return std::unique_ptr<NvjpgJpegEncoder>(new NvjpgJpegEncoder(*loaded, encoder));
}

NvjpgJpegEncoder::~NvjpgJpegEncoder() { plugin_.destroy(encoder_); }

bool NvjpgJpegEncoder::encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) {
    out.clear();
    if (bgr.empty() || bgr.type() != CV_8UC3) return false;
    const int width = bgr.cols & ~1;
    const int height = bgr.rows & ~1;
    if (width == 0 || height == 0) return false;

    // JPEG decoders read YCbCr as full range (JFIF). COLOR_BGR2YUV_I420 is limited range (Y in
    // 16-235), which decoded washed out: 6.2 mean error per channel on a gradient. So convert
    // to full-range YCrCb and subsample the chroma to 4:2:0 here.
    cv::cvtColor(bgr(cv::Rect(0, 0, width, height)), ycrcb_, cv::COLOR_BGR2YCrCb);
    cv::split(ycrcb_, planes_);
    i420_.create(height * 3 / 2, width, CV_8UC1);
    unsigned char *dst = i420_.ptr<unsigned char>();
    const size_t luma = static_cast<size_t>(width) * static_cast<size_t>(height);
    cv::Mat y(height, width, CV_8UC1, dst);
    cv::Mat cb(height / 2, width / 2, CV_8UC1, dst + luma);
    cv::Mat cr(height / 2, width / 2, CV_8UC1, dst + luma + luma / 4);
    planes_[0].copyTo(y);
    cv::resize(planes_[2], cb, cb.size(), 0, 0, cv::INTER_AREA);
    cv::resize(planes_[1], cr, cr.size(), 0, 0, cv::INTER_AREA);
    const unsigned char *jpeg = nullptr;
    const size_t size =
        plugin_.encode(encoder_, i420_.ptr<unsigned char>(), width, height, quality, &jpeg);
    if (size == 0) return false;
    const auto *begin = reinterpret_cast<const std::byte *>(jpeg);
    out.assign(begin, begin + size);
    return true;
}

}  // namespace auto_battlebot
