// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 13:13:12
// @file read_groundtruth.cpp
// @brief 
// 
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include <Eigen/Geometry>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>

namespace pt = boost::property_tree;

namespace adu {
namespace perception {

enum Type {
    smallMot = 0
};

struct Box {
    Eigen::AngleAxisd rotation_x;
    Eigen::AngleAxisd rotation_y;
    Eigen::AngleAxisd rotation_z;
    Eigen::AlignedBox3d bounding_box;
    std::string type;
public:
    Box(const pt::ptree& root); 

    std::string debug_string() const;
};

struct Label {
    std::string file_name;
    std::vector<Box> boxes;
public:
    Label(const std::string& file, const pt::ptree& root);
    std::string debug_string() const;
};

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
       << " box:(" << bounding_box.max() << ","<< bounding_box.min() << ")"
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

int main() {
    std::string file_name = "/home/erlangz/3D_point_cloud/0711/groundtruth/result.txt";
    //Read Data from line 
    std::ifstream ifs;
    ifs.open(file_name.c_str());
    if (!ifs) {
        std::cerr << "Open File:" << file_name << " failed." << std::endl;
        return -1;
    }
    //Parse Data from Columns
    std::string line;
    while (std::getline(ifs, line)) {

        std::vector<std::string> columns;
        boost::split(columns, line, boost::is_any_of(" \t"));
        std::string& pcd_file_name = columns[2];
        std::string& json_data = columns[3];

        std::stringstream json_stream;
        json_stream << json_data;

        pt::ptree root;
        pt::read_json(json_stream, root);
        adu::perception::Label label(pcd_file_name, root);
        std::cout << label.debug_string();
        break;
    }

    return 0;
}
