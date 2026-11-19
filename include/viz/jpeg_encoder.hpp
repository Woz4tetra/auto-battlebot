#pragma once

#include <atomic>
#include <cassert>
#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <string_view>
#include <thread>
#include <vector>

namespace auto_battlebot {

class JpegEncoderInterface {
   public:
    virtual ~JpegEncoderInterface() = default;
    bool encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) {
        const std::thread::id caller = std::this_thread::get_id();
        std::thread::id unowned{};
        owner_.compare_exchange_strong(unowned, caller);
        assert(owner_.load() == caller);
        return encode_on_owner_thread(bgr, quality, out);
    }
    virtual std::string_view name() const = 0;

   protected:
    virtual bool encode_on_owner_thread(const cv::Mat &bgr, int quality,
                                        std::vector<std::byte> &out) = 0;

   private:
    std::atomic<std::thread::id> owner_{};
};

class OpenCvJpegEncoder : public JpegEncoderInterface {
   public:
    std::string_view name() const override { return "opencv"; }

   protected:
    bool encode_on_owner_thread(const cv::Mat &bgr, int quality,
                                std::vector<std::byte> &out) override;

   private:
    std::vector<uint8_t> buffer_;
};

std::unique_ptr<JpegEncoderInterface> make_jpeg_encoder();

}  // namespace auto_battlebot
