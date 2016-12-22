// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 11:05:55
// @file show_point_cloud.cpp
// @brief 
// 
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "types.h"
#include "label_reader.h"

void viewOneOff(pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(0, 0, 0); // set background black.
}

int main() {
    //Read Point Cloud from File
    pcl::PCDReader file_reader;
    std::string file_name = "/home/erlangz/3D_point_cloud/0711/original_cloud/QB9178_12_1461753402_1461753702_3641.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Point Cloud MonochromeCloud(pcl::PointCloud<pcl::PointXYZ>)
    Eigen::Vector4f origin;          //sensor acquistion origin
    Eigen::Quaternionf orientation;  //the sensor acquistion orientation
    int file_version = 0;            //file version 
    int offset = 0;                  //[input] offset, you may change this when read from tar file.

    int ret = file_reader.read(file_name, *point_cloud, offset);
    if (ret != 0) {
        std::cerr << "file:" << file_name << " open failed. ret:" << ret << std::endl;
        return -1;
    }
    std::cout << "Open file:" << file_name << " file_version:" << file_version << std::endl;

    //Read Cubes from File.
    file_name = "/home/erlangz/3D_point_cloud/0711/groundtruth/result.txt";
    adu::perception::LabelsReader reader;
    if (!reader.init(file_name)) {
        std::cerr << "Open Label File:" << file_name << " failed." << std::endl;
        return -1;
    }

    //Show Point Cloud on the Screen
    pcl::visualization::CloudViewer cloud_viewer("cloud_viewer"); 
    cloud_viewer.showCloud(point_cloud);
    cloud_viewer.runOnVisualizationThreadOnce(viewOneOff);

    std::cin >> ret;
    return 0;
}
