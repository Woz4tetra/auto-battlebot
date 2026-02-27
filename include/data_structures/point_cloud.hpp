#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "header.hpp"

namespace auto_battlebot {
struct PointCloud {
    Header header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    PointCloud() : cloud(new pcl::PointCloud<pcl::PointXYZ>()) {}
};

}  // namespace auto_battlebot
