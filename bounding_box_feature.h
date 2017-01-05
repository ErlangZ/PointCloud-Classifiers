// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/29 17:57:00
// @file bouing_box_feature.h
// @brief 
#ifndef ADU_PERCEPTION_PCD_BOUING_BOX_FEATURE_H
#define ADU_PERCEPTION_PCD_BOUING_BOX_FEATURE_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace adu {
namespace perception {

class BoundingBoxFeature {
public:
    BoundingBoxFeature();
    bool compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                 std::vector<float>* bouding_box_features);
    bool min_max(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                 Eigen::Vector3f& min, Eigen::Vector3f& max);

};

} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_PCD_BOUING_BOX_FEATURE_H
// 

