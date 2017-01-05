// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/29 19:29:24
// @file bounding_box_feature.cpp
// @brief 
// 
#include "bounding_box_feature.h"
namespace adu {
namespace perception {

BoundingBoxFeature::BoundingBoxFeature() {

}

bool BoundingBoxFeature::min_max(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                                 Eigen::Vector3f& min, Eigen::Vector3f& max) {
    //Get AABB
    for (int i = 0; i < 3; i++) {
        min(i) = FLT_MAX;
        max(i) = -FLT_MAX; 
    }
    for (int i = 0; i < object->points.size(); i++) {
        if (object->points[i].x > max(0)) {
            max(0) = object->points[i].x;
        }
        if (object->points[i].x < min(0)) {
            min(0) = object->points[i].x;
        }
        if (object->points[i].y > max(1)) {
            max(1) = object->points[i].y;
        }
        if (object->points[i].y < min(1)) {
            min(1) = object->points[i].y;
        }
        if (object->points[i].z > max(2)) {
            max(2) = object->points[i].z;
        }
        if (object->points[i].z < min(2)) {
            min(2) = object->points[i].z;
        }
    }
    return true;
}

bool BoundingBoxFeature::compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                                 std::vector<float>* bounding_box_feature) {
    Eigen::Vector3f min; 
    Eigen::Vector3f max;

    if(!min_max(object, min, max)) {
        return false;
    }

    //Length, Width, Height 
    bounding_box_feature->resize(3, 0.0);
    for (int i = 0; i < 3; i++) {
        bounding_box_feature->at(i) = max(i) - min(i);
    }
    return true;
}

}
} // namespace adu
