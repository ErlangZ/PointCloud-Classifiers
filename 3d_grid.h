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

namespace adu {
namespace perception {

template<typename PointT>
class Grid {
public:
    Grid(){};
    void set_point_cloud_xyz_coord(const typename pcl::PointCloud<PointT>::Ptr object) {
        _point_cloud = object;
    }
    //@brief X,Y,Z -> Theta,Radius,Z
    const typename pcl::PointCloud<PointT>::Ptr get_point_cloud_cylindrical_coord() {
        assert(_point_cloud);
        typename pcl::PointCloud<PointT>::Ptr cylindrical_point_cloud(new typename pcl::PointCloud<PointT>);
        cylindrical_point_cloud->points.resize(_point_cloud->points.size());
        for (int i = 0; i < _point_cloud->points.size(); i++) {
            const PointT& point = _point_cloud->points[i];
            cylindrical_point_cloud->at(i).x = std::atan(point.y / point.x);
            cylindrical_point_cloud->at(i).y = std::sqrt(point.x * point.x + point.y * point.y);
            cylindrical_point_cloud->at(i).z = point.z;
        }
        return cylindrical_point_cloud;
    }
private:
    typename pcl::PointCloud<PointT>::Ptr _point_cloud;
};

}
}
#endif  // ADU_PERCEPTION_PCD_3D_GRID_H

