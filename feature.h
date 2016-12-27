// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/27 16:48:56
// @file feature.h
// @brief 
#ifndef ADU_PERCEPTION_PCD_FEATURE_H
#define ADU_PERCEPTION_PCD_FEATURE_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace adu {
namespace perception {

class HogFeature {
public:
    HogFeature();
    bool compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                 const int dim,
                 std::vector<float>* hog_feature);
private:
    cv::Mat get_image_in_dim(const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                             int dim1, int dim2);
    std::pair<double, double> find_min_max(const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                             int dim);
    int get_coord_on_image(const pcl::PointXYZ& point, 
                           int dim, double min, double max, int columns);
    double get_dim(const pcl::PointXYZ& point, int dim) {
        switch (dim) {
            case 0: return point.x;
            case 1: return point.y;
            case 2: return point.z;
            default: return NAN;
        }
    }
private:
    cv::Size _image_size;
    cv::HOGDescriptor _hog;
};

}
}
#endif  // ADU_PERCEPTION_PCD_FEATURE_H

