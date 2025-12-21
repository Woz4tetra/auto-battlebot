#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include "header.hpp"
#include "transform.hpp"
#include "point_cloud.hpp"
#include "enums/label.hpp"

namespace auto_battlebot
{
    struct Mask
    {
        Label label;
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
        Transform tf_camera_from_fieldcenter;
        SizeStamped size;
        PointCloud inlier_points;
    };

} // namespace auto_battlebot
