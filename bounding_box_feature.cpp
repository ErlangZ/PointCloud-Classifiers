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

bool BoundingBoxFeature::compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                                 std::vector<float>* bounding_box_feature) {
    //Get AABB
    float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
    float max_x = FLT_MIN, max_y = FLT_MIN, max_z = FLT_MIN;
    for (int i = 0; i < object->points.size(); i++) {
        if (object->points[i].x > max_x) {
            max_x = object->points[i].x;
        }
        if (object->points[i].x < min_x) {
            min_x = object->points[i].x;
        }
        if (object->points[i].y > max_y) {
            max_y = object->points[i].y;
        }
        if (object->points[i].y < min_y) {
            min_y = object->points[i].y;
        }
        if (object->points[i].z > max_z) {
            max_z = object->points[i].z;
        }
        if (object->points[i].z < min_z) {
            min_z = object->points[i].z;
        }
    }

    //Length, Width, Height 
    bounding_box_feature->resize(3, 0.0);
    bounding_box_feature->at(0) = max_x - min_x;
    bounding_box_feature->at(1) = max_y - min_y;
    bounding_box_feature->at(2) = max_z - min_z;
    return true;
}

}
} // namespace adu
