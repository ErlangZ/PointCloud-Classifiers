// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 13:13:12
// @file read_groundtruth.cpp
// @brief 
// 

#include "types.h"

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
