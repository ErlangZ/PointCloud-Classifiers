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
    _labels_file_name = file_name;
    //Read Data from line 
    std::ifstream ifs;
    ifs.open(_labels_file_name.c_str(), std::fstream::in);
    if (!ifs) {
        std::cerr << "Open File:" << _labels_file_name << " failed. e:" << strerror(errno) << std::endl;
        return false;
    }
    //Parse Data from Columns
    std::string line;
    while (std::getline(ifs, line)) {

        std::vector<std::string> columns;
        boost::split(columns, line, boost::is_any_of(" \t"));
        const std::string& pcd_file_name = columns[2].substr(8); //"./files/xxx" -> "xxx"
        std::string& json_data = columns[3];

        std::stringstream json_stream;
        json_stream << json_data;

        pt::ptree root;
        pt::read_json(json_stream, root);
        _labels[pcd_file_name] = Label::Ptr(new Label(pcd_file_name, root));
//        if (pcd_file_name == "QB9178_12_1461753402_1461753702_3641.pcd") {
//            std::cout << _labels[pcd_file_name]->debug_string();
//        }
//        std::cout << "LabelsReader pcd_file_name:" << pcd_file_name << std::endl;
    }
    return true;
}

} // namespace perception
} // namespace adu
