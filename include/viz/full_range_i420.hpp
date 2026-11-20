#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace auto_battlebot {

class FullRangeI420Converter {
   public:
    const cv::Mat *convert(const cv::Mat &bgr);
    int width() const { return width_; }
    int height() const { return height_; }

   private:
    int width_ = 0;
    int height_ = 0;
    cv::Mat ycrcb_;
    std::vector<cv::Mat> planes_;
    cv::Mat i420_;
};

}  // namespace auto_battlebot
