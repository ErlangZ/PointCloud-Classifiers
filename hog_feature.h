// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/27 16:48:56
// @file feature.h
// @brief 
#ifndef ADU_PERCEPTION_PCD_FEATURE_H
#define ADU_PERCEPTION_PCD_FEATURE_H

#include "types.h"

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace adu {
namespace perception {

class HogFeature {
public:
    HogFeature();
    bool compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                 std::vector<std::vector<float> >* hog_features);
    void serialize(std::ostream& os, 
                   const std::string& id,
                   const std::string& type, const std::vector<float>& features);
private:
    bool compute_hog(const cv::Mat image, std::vector<float>* hog_feature);
    //@brief Project PointCloud on Image
    cv::Mat get_image_in_dim(std::vector<unsigned char>& grid,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                             int dim1, int dim2);
    boost::shared_ptr<std::vector<unsigned char> > new_data() {
        int length = _image_size.height * _image_size.width;
        return boost::shared_ptr<std::vector<unsigned char> >(new std::vector<unsigned char>(length, 0));
    }
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

