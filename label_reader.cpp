// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 16:36:23
// @file label_reader.cpp
// @brief 
// 
#include "label_reader.h"

#include <exception>
#include <system_error>

namespace adu {
namespace perception {

bool LabelsReader::init(const std::string& file_name) {
    _file_name = file_name;
    //Read Data from line 
    std::ifstream ifs;
    ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        ifs.open(_file_name.c_str(), std::fstream::in);
    } catch (const std::exception& e) {
        std::cerr << "Open File:" << _file_name << " failed. e:" << e.what() << std::endl;
        return false;
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
        _labels.emplace_back(pcd_file_name, root);
    }
    return true;
}

} // namespace perception
} // namespace adu
