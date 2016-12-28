// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 16:13:51
// @file types.cpp
// @brief 

#include "types.h" 

#include <boost/typeof/typeof.hpp>
#include <boost/foreach.hpp>
#include <pcl/filters/extract_indices.h>

namespace adu {
namespace perception {

Box::Box(int id_, const pt::ptree& root) {
    id = id_;
    //Rotation 
    BOOST_AUTO(r, root.get_child("rotation"));
    //phi - Z axis; theta - X axis; psi - Y axis
    rotation_z = Eigen::AngleAxisd(r.get<double>("phi"), Eigen::Vector3d::UnitZ());
    rotation_x = Eigen::AngleAxisd(r.get<double>("theta"), Eigen::Vector3d::UnitX());
    rotation_y = Eigen::AngleAxisd(r.get<double>("psi"), Eigen::Vector3d::UnitY());

    std::vector<double> size;
    BOOST_FOREACH(const pt::ptree::value_type& item, root.get_child("size")) {
        size.push_back(item.second.get_value<double>());
    }
    //AlignedBox 
    BOOST_AUTO(center, root.get_child("position"));
    bounding_box = Eigen::AlignedBox3d(Eigen::Vector3d(center.get<double>("x") - size[0]/2, center.get<double>("y") - size[1]/2, center.get<double>("z") - size[2]/2),
                                       Eigen::Vector3d(center.get<double>("x") + size[0]/2, center.get<double>("y") + size[1]/2, center.get<double>("z") + size[2]/2));

    //Type
    const std::string& type_str = root.get<std::string>("type");
    if (type_str == "smallMot") {
        type = smallMot;
    } else if (type_str == "pedestrian") {
        type = pedestrian;
    } else if (type_str == "bigMot") {
        type = bigMot;
    } else if (type_str == "midMot") {
        type = midMot;
    } else if (type_str == "nonMot") {
        type = nonMot;
    } else if (type_str == "unknow") {
        type = unknown;
    } else if (type_str == "cluster") {
        type = cluster;
    } else {
        std::cout << type_str << std::endl;
    }
}

void Box::show(pcl::visualization::PCLVisualizer& viewer) {
    Eigen::Vector3f color = get_color();
    viewer.addCube(bounding_box.min().x(), bounding_box.max().x(), 
                   bounding_box.min().y(), bounding_box.max().y(), 
                   bounding_box.min().z(), bounding_box.max().z(), 
                   color(0), color(1), color(2),
                   id_str());
}

std::string Box::type_str() const {
    switch (type) {
        case smallMot: return "smallMot";
        case midMot: return "midMot";
        case bigMot: return "bigMot"; 
        case nonMot: return "nonMot";
        case pedestrian: return "pedestrian";
        case unknown: return "unknown";
    }
}
const std::string Box::id_str() const {
    std::stringstream ss;
    ss << type << "-" << id;
    return ss.str();
}

Eigen::Vector3f Box::get_color() const {
    switch (type) {
        case smallMot: return Eigen::Vector3f(0.0, 1.0, 0.0);
        case midMot: return Eigen::Vector3f(0.0, 0.8, 0.0);
        case bigMot: return Eigen::Vector3f(0.0, 0.9, 0.0);
        case nonMot: return Eigen::Vector3f(0.0, 0.7, 0.0);
        case pedestrian: return Eigen::Vector3f(1.0, 0.0, 0.0);
        case unknown: return Eigen::Vector3f(0.0, 0.0, 1.0);
    }
}

std::string Box::debug_string() const {
    std::stringstream ss;
    ss << "type:" << type 
       << " box:(" << bounding_box.max().transpose() << ","<< bounding_box.min().transpose() << ")";
    return ss.str();
}

pcl::PointIndices::Ptr BoxFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, const Box& box) {
    pcl::PointIndices::Ptr indice(new pcl::PointIndices);
    for (size_t i = 0; i < point_cloud->size(); i++) {
        if (box.bounding_box.exteriorDistance(Eigen::Vector3d(point_cloud->at(i).x, point_cloud->at(i).y, point_cloud->at(i).z)) == 0) {
            indice->indices.push_back(i);
        }
    }
    return indice;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BoxFilter::filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, 
        const pcl::PointIndices::Ptr& point_indice) {
    pcl::ExtractIndices<pcl::PointXYZ> eifilter(true);
    eifilter.setInputCloud(point_cloud);
    eifilter.setIndices(point_indice);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_point(new pcl::PointCloud<pcl::PointXYZ>);
    eifilter.filter(*output_point);
    return output_point;
}

Label::Label(const std::string& file, const pt::ptree& root) {
    file_name = file; 
    int i = 0;
    BOOST_FOREACH(const pt::ptree::value_type& result, root.get_child("result")) {
        boxes.push_back(Box::Ptr(new Box(i++, result.second))); 
    }
}

std::string Label::debug_string() const {
    std::stringstream ss;
    ss << "file_name:" << file_name 
       << "Boxes:(" << boxes.size() << ")";
    for (size_t i = 0; i < boxes.size(); i++) {
        ss << "[" << boxes[i]->debug_string() << "]";
    }
    return ss.str();
}

} // namespace perception
} // namespace adu
