#include "foxglove_adapters/image.hpp"

#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "foxglove_adapters/common.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::CompressedImage to_compressed_image(const RgbImage &rgb_image) {
    foxglove::schemas::CompressedImage image;
    image.timestamp = to_timestamp(rgb_image.header.stamp);
    image.frame_id = frame_id_string(rgb_image.header.frame_id);
    image.format = "jpeg";
    if (rgb_image.image.empty()) return image;

    std::vector<uint8_t> buffer;
    cv::imencode(".jpg", rgb_image.image, buffer);
    image.data.assign(reinterpret_cast<const std::byte *>(buffer.data()),
                      reinterpret_cast<const std::byte *>(buffer.data() + buffer.size()));
    return image;
}

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
