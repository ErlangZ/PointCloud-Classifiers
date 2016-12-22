// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 16:13:51
// @file types.cpp
// @brief 

#include "types.h" 

namespace adu {
namespace perception {

Box::Box(const pt::ptree& root) {
    //Rotation 
    const auto& r = root.get_child("rotation");
    //phi - Z axis; theta - X axis; psi - Y axis
    rotation_z = Eigen::AngleAxisd(r.get<double>("phi"), Eigen::Vector3d::UnitZ());
    rotation_x = Eigen::AngleAxisd(r.get<double>("theta"), Eigen::Vector3d::UnitX());
    rotation_y = Eigen::AngleAxisd(r.get<double>("psi"), Eigen::Vector3d::UnitY());

    //AlignedBox 
    const auto& t = root.get_child("position");
    Eigen::Translation3d top = Eigen::Translation3d(t.get<double>("x"), t.get<double>("y"), t.get<double>("z"));
    std::vector<double> size;
    for (const auto& item : root.get_child("size")) {
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
       << " (z:" << rotation_z.angle() << ")";
    return ss.str();
}

Label::Label(const std::string& file, const pt::ptree& root) {
    file_name = file;
    for (const auto& result : root.get_child("result")) {
        boxes.emplace_back(result.second); 
    }
}

std::string Label::debug_string() const {
    std::stringstream ss;
    ss << "file_name:" << file_name 
       << "Boxes:(" << boxes.size() << ")";
    for (const auto& box : boxes) {
        ss << "[" << box.debug_string() << "]";
    }
    return ss.str();
}
} // namespace perception
} // namespace adu
