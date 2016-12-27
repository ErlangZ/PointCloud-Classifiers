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

    //AlignedBox 
    BOOST_AUTO(t, root.get_child("position"));
    Eigen::Translation3d top = Eigen::Translation3d(t.get<double>("x"), t.get<double>("y"), t.get<double>("z"));
    std::vector<double> size;
    BOOST_FOREACH(const pt::ptree::value_type& item, root.get_child("size")) {
        size.push_back(item.second.get_value<double>());
    }
    Eigen::Translation3d bottom = Eigen::Translation3d(top.x() + size[1], top.y() + size[2], top.z() + size[0]);
    bounding_box = Eigen::AlignedBox3d(Eigen::Affine3d(top).matrix().block<3, 1>(0, 3), 
                                       Eigen::Affine3d(bottom).matrix().block<3, 1>(0, 3));

    //Type
    type = root.get<std::string>("type");
}

std::string Box::debug_string() const {
    std::stringstream ss;
    ss << "type:" << type 
       << " box:(" << bounding_box.max().transpose() << ","<< bounding_box.min().transpose() << ")"
       << " rotation:(x:" << rotation_x.angle() << "),"
       << " (y:" << rotation_y.angle() << "),"
       << " (z:" << rotation_z.angle() << ")"
       << "T:" << translation().transpose() << " R:" << rotation().matrix()
       << " width:" << width() << " depth:" << depth() << " height:" << height();

    return ss.str();
}

pcl::PointIndices::Ptr BoxFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, const Box& box) {
    pcl::PointIndices::Ptr indice(new pcl::PointIndices);
    for (size_t i = 0; i < point_cloud->size(); i++) {
        BOOST_AUTO(point, point_cloud->at(i));
        //std::cout << "exteriorDistance:" << box.bounding_box.exteriorDistance(Eigen::Vector3d(point.x, point.y, point.z)) << std::endl;
        if (box.bounding_box.exteriorDistance(Eigen::Vector3d(point.x, point.y, point.z)) == 0) {
            //the point in the bounding_box
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
