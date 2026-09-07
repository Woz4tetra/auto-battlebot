#pragma once

#include <foxglove/schemas.hpp>

#include "data_structures/image.hpp"

namespace auto_battlebot {
namespace foxglove_adapters {

/** JPEG-encode an RGB image into a foxglove.CompressedImage. An empty image yields an empty
 *  payload with the header filled in. */
foxglove::schemas::CompressedImage to_compressed_image(const RgbImage &rgb_image);

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
