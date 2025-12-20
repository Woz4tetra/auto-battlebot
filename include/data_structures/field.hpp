#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include "header.hpp"
#include "transform.hpp"

namespace auto_battlebot
{
    struct Mask
    {
        std::string label;
        cv::Mat mask;
    };

    struct FieldMaskStamped
    {
        Header header;
        Mask mask;
    };

    struct Size
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct SizeStamped
    {
        Header header;
        Size size;
    };

    struct FieldDescription
    {
        Header header;
        Transform tf_fieldcenter_from_camera;
        SizeStamped size;
    };

} // namespace auto_battlebot
