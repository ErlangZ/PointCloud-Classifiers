// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 11:05:55
// @file show_point_cloud.cpp
// @brief 
// 
#include <iostream>
#include <string>

#include <pcl/io/file_io.h>
namespace adu {
namespace perception {

int main() {
    pcl::PCDReader file_reader;
    std::string file_name = "/home/erlangz/3D_point_cloud/0711/original_cloud/QB9178_12_1461753402_1461753702_3641.pcd";
    pcl::PCLPointCloud2 point_cloud; //Point Cloud
    Eigen::Vector4f origin;          //sensor acquistion origin
    Eigen::Quaternionf orientation;  //the sensor acquistion orientation
    int file_version = 0;            //file version 
    int offset = 0;                  //[input] offset, you may change this when read from tar file.

    int ret = file_reader.read(file_name, point_cloud, origin, orientation, file_version, offset);
    if (ret != 0) {
        std::cerr << "file:" << file_name << " open failed. ret:" << ret << std::endl;
        return -1;
    }
    
    std::cout << "Open file:" << file_name << " file_version:" << file_version << std::endl;

    return 0;
}
}
}
