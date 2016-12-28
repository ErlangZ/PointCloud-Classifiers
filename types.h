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
#include <pcl/visualization/cloud_viewer.h>
#include <boost/unordered_map.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>

namespace pt = boost::property_tree;

namespace adu {
namespace perception {

enum Type {
    unknown = 0,
    nonMot = 1, 
    bigMot = 2,
    midMot = 3,
    smallMot = 4,
    pedestrian = 5,
    cluster = 6
};

class Box {
public:
    Box(int id, const pt::ptree& root); 
    void show(pcl::visualization::PCLVisualizer& viewer);
    const std::string id_str() const;
    std::string type_str() const;
    std::string debug_string() const;
private:
    //type -> RGB 
    Eigen::Vector3f get_color() const;
private:
    Eigen::AngleAxisd rotation_x;
    Eigen::AngleAxisd rotation_y;
    Eigen::AngleAxisd rotation_z;
    Type type;
    int id;
    Eigen::AlignedBox3d bounding_box;
public:
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

