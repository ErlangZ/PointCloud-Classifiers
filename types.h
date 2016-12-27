// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 16:09:08
// @file types.h
// @brief 
#ifndef ADU_PERCEPTION_PCD_TYPES_H
#define ADU_PERCEPTION_PCD_TYPES_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <boost/unordered_map.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>

namespace pt = boost::property_tree;

namespace adu {
namespace perception {

enum Type {
    smallMot = 0
};

class Box {
    Eigen::AngleAxisd rotation_x;
    Eigen::AngleAxisd rotation_y;
    Eigen::AngleAxisd rotation_z;
    std::string type;
    int id;
public:
    Eigen::AlignedBox3d bounding_box;
    Box(int id, const pt::ptree& root); 
    const Eigen::Vector3d translation() const {
        return bounding_box.min();
    }
    const Eigen::Quaterniond rotation() const {
        return rotation_x * rotation_y * rotation_z;
    }
    double depth() const {
        return (bounding_box.max() - bounding_box.min())(0);
    }
    double width() const {
        return (bounding_box.max() - bounding_box.min())(1);
    }
    double height() const {
        return (bounding_box.max() - bounding_box.min())(2);
    }
    const std::string id_str() const {
        std::stringstream ss;
        ss << type << "-" << id;
        return ss.str();
    }
    std::string debug_string() const;
    typedef boost::shared_ptr<Box> Ptr;
    friend class BoxFilter;
};

class BoxFilter {
public:
    //filter the point cloud
    static pcl::PointIndices::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud,  const Box& box);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, 
                                                      const pcl::PointIndices::Ptr& point_indice);
};

class Label {
    std::string file_name;
public:
    std::vector<Box::Ptr> boxes;
    Label(const std::string& file, const pt::ptree& root);
    const std::vector<Box::Ptr>& get(const std::string file) const {
        return boxes;
    }
    std::string debug_string() const;
    typedef boost::shared_ptr<Label> Ptr;
};

} //namespace perception
} //namespace adu
#endif  // ADU_PERCEPTION_PCD_TYPES_H
// 

