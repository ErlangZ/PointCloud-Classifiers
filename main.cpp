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

int main() {
    using adu::perception::Box;
    std::string data_root = "/home/erlangz/3D_point_cloud/0711/original_cloud/";

    //Read Point Cloud from File
    pcl::PCDReader file_reader;
    std::string pcd_file_name =  "QB9178_12_1461753402_1461753702_3641.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Point Cloud MonochromeCloud(pcl::PointCloud<pcl::PointXYZ>)
    Eigen::Vector4f origin;          //sensor acquistion origin
    Eigen::Quaternionf orientation;  //the sensor acquistion orientation
    int file_version = 0;            //file version 
    int offset = 0;                  //[input] offset, you may change this when read from tar file.

    int ret = file_reader.read(data_root + pcd_file_name, *point_cloud, offset);
    if (ret != 0) {
        std::cerr << "file:" << data_root + pcd_file_name << " open failed. ret:" << ret << std::endl;
        return -1;
    }
    std::cout << "Open file:" << data_root + pcd_file_name << " file_version:" << file_version << std::endl;

    //Read Cubes from File.
    std::string label_file_name = "/home/erlangz/3D_point_cloud/0711/groundtruth/result.txt";
    adu::perception::LabelsReader labels_reader;
    if (!labels_reader.init(label_file_name)) {
        std::cerr << "Open Label File:" << label_file_name << " failed." << std::endl;
        return -1;
    }
    
    BOOST_AUTO(label, labels_reader.get(pcd_file_name));
    //Show Point Cloud on the Screen
    pcl::visualization::CloudViewer cloud_viewer("cloud_viewer"); 

    //Filter Points in Box
    pcl::PointIndices::Ptr all_in_boxes = pcl::PointIndices::Ptr(new pcl::PointIndices);
    for (size_t i = 0; i < label->boxes.size(); i++) {
        const Box::Ptr& box = label->boxes[i];
        BOOST_AUTO(point, adu::perception::BoxFilter::filter(point_cloud, *box));
        all_in_boxes->indices.insert(all_in_boxes->indices.end(), point->indices.begin(), point->indices.end());
    }
    point_cloud = adu::perception::BoxFilter::filter(point_cloud, all_in_boxes);
   

    cloud_viewer.showCloud(point_cloud);
    /*
    cloud_viewer.runOnVisualizationThreadOnce([&label,&pcd_file_name](pcl::visualization::PCLVisualizer& viewer) {
        viewer.setBackgroundColor(0, 0, 0); // set background black.
        if (!label) {
            std::cerr << "Labels in pcd_file:" << pcd_file_name << " open failed."<< std::endl;
            return;
        }
        for (size_t i = 0; i < label->boxes.size(); i++) {
            const Box::Ptr& box = label->boxes[i];
            viewer.addCube(box->translation().cast<float>(), box->rotation().cast<float>(), box->width(), box->height(), box->depth(), box->id_str());
            std::cout << "Add Box:" << box->debug_string() << std::endl;
        }
    });
    */
    
    while(!cloud_viewer.wasStopped()) {
    }
    return 0;
}
