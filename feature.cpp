// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/27 17:10:28
// @file feature.cpp
// @brief 
// 
#include "feature.h"

#include <vector>

namespace adu {
namespace perception {

HogFeature::HogFeature() : 
    _image_size(1024, 1024) {
    
}

cv::Mat HogFeature::get_image_in_dim(std::vector<double>& grid,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                                     int dim1, int dim2) {
    std::pair<double, double> u_min_max = find_min_max(object, dim1);
    std::pair<double, double> v_min_max = find_min_max(object, dim2);
    for (size_t i = 0; i < object->points.size(); i++) {
        int u = get_coord_on_image(object->points[i], dim1, u_min_max.first, u_min_max.second,_image_size.height);
        int v = get_coord_on_image(object->points[i], dim2, v_min_max.first, v_min_max.second, _image_size.width);
        grid[u * _image_size.height + v] += 1.0;
    }
    std::vector<double>::iterator max_value = std::max_element(grid.begin(), grid.end());
    for (size_t i = 0; i < grid.size(); i++) {
        grid[i] = grid[i] / (*max_value) * 255.0;
    }

    return cv::Mat(_image_size.height, _image_size.width, CV_64FC1, grid.data());
}

std::pair<double, double> HogFeature::find_min_max(const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                                                   int dim) {
    std::pair<double, double> result;
    double& min = result.first;
    double& max = result.second;
    min = FLT_MAX;
    max = FLT_MIN;
    for (size_t i = 0; i < object->points.size(); i++) {
        const pcl::PointXYZ& point = object->points[i];
        if (min > get_dim(point, dim)) {
            min = get_dim(point, dim);
        }
        if (max < get_dim(point, dim)) {
            max = get_dim(point, dim);
        }
    }
    return result;
}

int HogFeature::get_coord_on_image(const pcl::PointXYZ& point, 
                                   int dim, double min, double max, int columns) {
    double value = get_dim(point, dim);
    return (value - min) / (max - min) * (columns-1); 
}


}
}
