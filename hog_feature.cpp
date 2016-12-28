// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/27 17:10:28
// @file feature.cpp
// @brief 
// 
#include "hog_feature.h"

#include <vector>

namespace adu {
namespace perception {

HogFeature::HogFeature() : 
    _image_size(300, 300),
    //  winSize, BlockSize, BlockStride, CellSize, CellBins
    _hog(cv::Size(60, 60), cv::Size(30, 30), cv::Size(10, 10), cv::Size(10, 10), 6) {
    
}

bool HogFeature::compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                         std::vector<std::vector<float> >* hog_features) {
    if (object->points.size() < 10) {
        return false;
    }
    //X-axis, Y-axis, Z-axis 
    hog_features->resize(3);

    boost::shared_ptr<std::vector<unsigned char> > data2 = new_data();
    boost::shared_ptr<std::vector<unsigned char> > data3 = new_data();
#pragma omp parallel for
    for (size_t i = 0; i < 3; i++) {
        boost::shared_ptr<std::vector<unsigned char> > data = new_data();
        if (i == 0) {
            cv::Mat image = get_image_in_dim(*data, object, 1, 2);
            _hog.compute(image, hog_features->at(i));
        } else if (i == 1) {
            cv::Mat image = get_image_in_dim(*data, object, 0, 2);
            _hog.compute(image, hog_features->at(i));
        } else {
            cv::Mat image = get_image_in_dim(*data, object, 0, 1);
            _hog.compute(image, hog_features->at(i));
        }
    }
#ifdef SHOW_XYZ_PRO_IMAGE
    cv::imshow("X-axis", x_image);
    cv::imshow("Y-axis", y_image);
    cv::imshow("Z-axis", z_image);
    cv::waitKey(0); 
#endif
    return true;
}

cv::Mat HogFeature::get_image_in_dim(std::vector<unsigned char>& grid,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr object, 
                                     int dim1, int dim2) {
    std::vector<double> temp_grid(grid.size(), 0.0);
    std::pair<double, double> u_min_max = find_min_max(object, dim1);
    std::pair<double, double> v_min_max = find_min_max(object, dim2);

    for (size_t i = 0; i < object->points.size(); i++) {
        int u = get_coord_on_image(object->points[i], dim1, u_min_max.first, u_min_max.second,_image_size.height);
        int v = get_coord_on_image(object->points[i], dim2, v_min_max.first, v_min_max.second, _image_size.width);
        temp_grid[u * _image_size.height + v] += 1.0;
    }
    std::vector<double>::iterator max_value = std::max_element(temp_grid.begin(), temp_grid.end());
#pragma omp parallel for
    for (size_t i = 0; i < temp_grid.size(); i++) {
        grid[i] = temp_grid[i] / (*max_value) * 255.0;
    }

    return cv::Mat(_image_size.height, _image_size.width, CV_8UC1, grid.data());
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

void HogFeature::serialize(std::ostream& os, const std::string& type, 
               const std::vector<float>& features) {
    os << type << "\t";
    for (int i = 0; i < features.size(); i++) {
        os << features[i];
        if (i != features.size() - 1) {
            os << "\t";
        } else {
            os << std::endl;
        }
    }
}

}
}
