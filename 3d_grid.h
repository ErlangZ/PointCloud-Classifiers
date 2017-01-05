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
class InnerGrid {
public:
    InnerGrid(const typename pcl::PointCloud<PointT>::Ptr point_cloud,
              const Eigen::Vector3f& min, const Eigen::Vector3f& max) {
        const float X_resolution = 0.08 * (2 * M_PI)/180;
        const float Y_resolution = 0.1;
        const float Z_resolution = 0.1; 
        _row = std::ceil((max(0) - min(0)) / X_resolution) + 1;
        _col = std::ceil((max(1) - min(1)) / Y_resolution) + 1;
        _height = std::ceil((max(2) - min(2))) / Z_resolution + 1;
        for (int i = 0; i < _row; i++) {
            _data.push_back(Eigen::MatrixXf::Zero(_col, _height));
        }
      
        for (int i = 0; i < point_cloud->points.size(); i++) {
            const PointT& point = point_cloud->points[i];
            int row = (point.x - min(0)) / X_resolution;
            int col = (point.y - min(1)) / Y_resolution;
            int height = (point.z - min(2)) / Z_resolution;
            //std::cout << row << ":" << col << ":" << height << " point:"  << point << " " << _row << ","  << _col << "," << _height << ","<< std::endl;
            _data[row](col, height) += 1;
        }
    }
    const std::string debug_string() {
        std::stringstream ss;
        ss << "InnerGrid has:[" << _row << "X" << _col << "X" << _height << "] in all grids_num:" << size()
           << ", Point mean:" << mean() << ", Point max:" << max();
        
        std::vector<int> count_num(10, 0);
        count(&count_num);
        ss << " Distribution:";
        for (int i = 0; i < count_num.size(); i++) {
            ss << (float)(count_num[i]) / size() * 100 << "% ";
        }
        return ss.str();
    }
    int size() const {
        return _row * _col * _height;
    }
    float mean() const {
        float mean = 0.0;
        for (int i = 0; i < _data.size(); i++) {
            mean += _data[i].mean();
        }
        mean /= _row;
    }
    float max() const {
        float max = -FLT_MAX;
        for (int i = 0; i < _data.size(); i++) {
            if (max < _data[i].maxCoeff()) {
                max = _data[i].maxCoeff();
            }
        }
        return max;
    }
    void count(std::vector<int>* count_num) {
        int max_value = count_num->size() - 1;
        for (int i = 0; i < _row; i++) {
            for (int j = 0; j < _col; j++) {
                for (int k = 0; k < _height; k++) {
                    int num = _data[i](j, k);
                    count_num->at(std::min(max_value, num)) ++;
                }
            }
        }
    }
private:
    int _row; 
    int _col;
    int _height;
    std::vector<Eigen::MatrixXf> _data;
public:
    typedef boost::shared_ptr<InnerGrid> Ptr;
};

template<typename PointT>
class Grid {
public:
    Grid(){};

    void set_point_cloud_xyz_coord(const typename pcl::PointCloud<PointT>::Ptr object) {
        _point_cloud = xyz_coord_to_cylindrical_coord(object);
        BoundingBoxFeature bounding_box_features;
        bounding_box_features.min_max(_point_cloud, _min, _max);
        _grid = typename InnerGrid<PointT>::Ptr(new InnerGrid<PointT>(_point_cloud, _min, _max));
    }

    typename pcl::PointCloud<PointT>::Ptr cylindrical_coord_point_cloud() const {
        return _point_cloud;
    }

    std::string debug_string() const {
        std::stringstream ss;
        ss << "Point Cloud has:" << _point_cloud->points.size() << " points."
           << "BoundingBox:" << _min.transpose() << "," << _max.transpose() 
           << " " << _grid->debug_string();
        return ss.str();
    }

private:

    //@brief X,Y,Z -> Theta,Radius,Z
    const typename pcl::PointCloud<PointT>::Ptr xyz_coord_to_cylindrical_coord(const typename pcl::PointCloud<PointT>::Ptr xyz_point_cloud) {
        typename pcl::PointCloud<PointT>::Ptr cylindrical_point_cloud(new typename pcl::PointCloud<PointT>);
        cylindrical_point_cloud->points.resize(xyz_point_cloud->points.size());
        for (int i = 0; i < xyz_point_cloud->points.size(); i++) {
            const PointT& point = xyz_point_cloud->points[i];
            if (point.y > 0) {//1-2 quadrant
                cylindrical_point_cloud->at(i).x = std::atan(point.y / point.x) - M_PI/2;
            } else {
                cylindrical_point_cloud->at(i).x = std::atan(point.y / point.x) + M_PI/2;
            }
            cylindrical_point_cloud->at(i).y = std::sqrt(point.x * point.x + point.y * point.y);
            cylindrical_point_cloud->at(i).z = point.z;
        }
        return cylindrical_point_cloud;
    }

private:
    typename pcl::PointCloud<PointT>::Ptr _point_cloud;
    typename InnerGrid<PointT>::Ptr       _grid;
    Eigen::Vector3f _min; //0-Theta, 1-Radius, 2-Z
    Eigen::Vector3f _max; //0-Theta, 1-Radius, 2-Z
};

}
}
#endif  // ADU_PERCEPTION_PCD_3D_GRID_H

