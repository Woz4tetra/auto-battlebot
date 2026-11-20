#include "viz/full_range_i420.hpp"

#include <opencv2/imgproc.hpp>

namespace auto_battlebot {

const cv::Mat *FullRangeI420Converter::convert(const cv::Mat &bgr) {
    if (bgr.empty() || bgr.type() != CV_8UC3) return nullptr;
    width_ = bgr.cols & ~1;
    height_ = bgr.rows & ~1;
    if (width_ == 0 || height_ == 0) return nullptr;

    cv::cvtColor(bgr(cv::Rect(0, 0, width_, height_)), ycrcb_, cv::COLOR_BGR2YCrCb);
    cv::split(ycrcb_, planes_);
    i420_.create(height_ * 3 / 2, width_, CV_8UC1);
    unsigned char *dst = i420_.ptr<unsigned char>();
    const size_t luma = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    cv::Mat y(height_, width_, CV_8UC1, dst);
    cv::Mat cb(height_ / 2, width_ / 2, CV_8UC1, dst + luma);
    cv::Mat cr(height_ / 2, width_ / 2, CV_8UC1, dst + luma + luma / 4);
    planes_[0].copyTo(y);
    cv::resize(planes_[2], cb, cb.size(), 0, 0, cv::INTER_AREA);
    cv::resize(planes_[1], cr, cr.size(), 0, 0, cv::INTER_AREA);
    return &i420_;
}

}  // namespace auto_battlebot
