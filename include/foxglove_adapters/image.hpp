#pragma once

#include <foxglove/schemas.hpp>

#include "data_structures/image.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

foxglove::schemas::CompressedImage to_compressed_image(const RgbImage &rgb_image);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
