#include "viz/nvjpg_jpeg_encoder.hpp"

#include <dlfcn.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <optional>
#include <string>

namespace auto_battlebot {

const NvjpgJpegEncoder::Plugin *NvjpgJpegEncoder::plugin() {
    static const std::optional<Plugin> loaded = []() -> std::optional<Plugin> {
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

bool NvjpgJpegEncoder::encode_on_owner_thread(const cv::Mat &bgr, int quality,
                                              std::vector<std::byte> &out) {
    out.clear();
    const cv::Mat *i420 = converter_.convert(bgr);
    if (i420 == nullptr) return false;
    const unsigned char *jpeg = nullptr;
    const size_t size = plugin_.encode(encoder_, i420->ptr<unsigned char>(), converter_.width(),
                                       converter_.height(), quality, &jpeg);
    if (size == 0) return false;
    const auto *begin = reinterpret_cast<const std::byte *>(jpeg);
    out.assign(begin, begin + size);
    return true;
}

}  // namespace auto_battlebot
