// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2017/01/05 10:15:49
// @file 3d_grid.h
// @brief 
#ifndef ADU_PERCEPTION_PCD_3D_GRID_H
#define ADU_PERCEPTION_PCD_3D_GRID_H

#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "bounding_box_feature.h"

namespace adu {
namespace perception {

template<typename PointT>
class Grid {
public:
    Grid(){};

    void set_point_cloud_xyz_coord(const typename pcl::PointCloud<PointT>::Ptr object) {
        _point_cloud = xyz_coord_to_cylindrical_coord(object);
        BoundingBoxFeature bounding_box_features;
        bounding_box_features.min_max(_point_cloud, _min, _max);
    }
    typename pcl::PointCloud<PointT>::Ptr cylindrical_coord_point_cloud() const {
        return _point_cloud;
    }

    std::string debug_string() const {
        std::stringstream ss;
        ss << "Point Cloud has:" << _point_cloud->points.size() << " points."
           << "BoundingBox:" << _min.transpose() << "," << _max.transpose();
        return ss.str();
    }
private:
    //@brief X,Y,Z -> Theta,Radius,Z
    const typename pcl::PointCloud<PointT>::Ptr xyz_coord_to_cylindrical_coord(const typename pcl::PointCloud<PointT>::Ptr xyz_point_cloud) {
        typename pcl::PointCloud<PointT>::Ptr cylindrical_point_cloud(new typename pcl::PointCloud<PointT>);
        cylindrical_point_cloud->points.resize(xyz_point_cloud->points.size());
        for (int i = 0; i < xyz_point_cloud->points.size(); i++) {
            const PointT& point = xyz_point_cloud->points[i];
            cylindrical_point_cloud->at(i).x = std::atan(point.y / point.x);
            cylindrical_point_cloud->at(i).y = std::sqrt(point.x * point.x + point.y * point.y);
            cylindrical_point_cloud->at(i).z = point.z;
        }
        return cylindrical_point_cloud;
    }
private:
    typename pcl::PointCloud<PointT>::Ptr _point_cloud;
    Eigen::Vector3f _min; //0-Theta, 1-Radius, 2-Z
    Eigen::Vector3f _max; //0-Theta, 1-Radius, 2-Z
};

}
}
#endif  // ADU_PERCEPTION_PCD_3D_GRID_H

