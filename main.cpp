// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 11:05:55
// @file show_point_cloud.cpp
// @brief 
// 
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "hog_feature.h"
#include "types.h"
#include "label_reader.h"

using adu::perception::Box;
using adu::perception::Label;

void show_box(const Label::Ptr& label, pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(0, 0, 0); // set background black.
    for (size_t i = 0; i < label->boxes.size(); i++) {
        const Box::Ptr& box = label->boxes[i];
        box->show(viewer);
        std::cout << "Add Box:" << box->debug_string() << std::endl;
    }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr read_pcd(const std::string& pcd_file_name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Point Cloud MonochromeCloud(pcl::PointCloud<pcl::PointXYZ>)
    Eigen::Vector4f origin;          //sensor acquistion origin
    Eigen::Quaternionf orientation;  //the sensor acquistion orientation
    int file_version = 0;            //file version 
    int offset = 0;                  //[input] offset, you may change this when read from tar file.

    //Read Point Cloud from File
    pcl::PCDReader file_reader;
    int ret = file_reader.read(pcd_file_name, *point_cloud, offset);
    if (ret != 0) {
        std::cerr << "file:" << pcd_file_name << " open failed. ret:" << ret << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr();
    }
    std::cout << "Open file:" << pcd_file_name << " file_version:" << file_version << std::endl;
    return point_cloud;
}

void draw_point_cloud_and_bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const Label::Ptr label) {
    //Show Point Cloud on the Screen
    pcl::visualization::CloudViewer cloud_viewer("cloud_viewer"); 

    //Filter Points in Box
    pcl::PointIndices::Ptr all_in_boxes = pcl::PointIndices::Ptr(new pcl::PointIndices);
    for (size_t i = 0; i < label->boxes.size(); i++) {
        const Box::Ptr& box = label->boxes[i];
        BOOST_AUTO(object, adu::perception::BoxFilter::filter(point_cloud, *box));
        all_in_boxes->indices.insert(all_in_boxes->indices.end(), object->indices.begin(), object->indices.end());
    }
    std::cout << "Point Num:" << all_in_boxes->indices.size() << std::endl;
    point_cloud = adu::perception::BoxFilter::filter(point_cloud, all_in_boxes);
    cloud_viewer.showCloud(point_cloud);
    cloud_viewer.runOnVisualizationThreadOnce(boost::bind(&show_box, label, _1));
    
    while(!cloud_viewer.wasStopped()) {
    }
}

bool draw_hog_features(std::vector<boost::shared_ptr<std::ofstream> >& oss, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const Label::Ptr label) {
    adu::perception::HogFeature hog_feature;
    for (size_t i = 0; i < label->boxes.size(); i++) {
        const Box::Ptr& box = label->boxes[i];
        BOOST_AUTO(object_indices, adu::perception::BoxFilter::filter(point_cloud, *box));
        BOOST_AUTO(object, adu::perception::BoxFilter::filter(point_cloud, object_indices));
        
        std::vector<std::vector<float> > features;
        if (!hog_feature.compute(object, &features)) {
            std::cerr << "Compute Object HogFeature failed, Box:" << box->debug_string() << std::endl;
            return false;
        }
#pragma omp parallel for
        for (size_t i = 0; i < features.size(); i++) {
            hog_feature.serialize(*oss[i], box->id_str(), box->type_str(), features[i]);
        }
    }
    return true;
}

int main() {
    std::string data_root = "/home/erlangz/3D_point_cloud/0711/original_cloud/";
    //Read Cubes from File.
    std::string label_file_name = "/home/erlangz/3D_point_cloud/0711/groundtruth/result.txt";
    adu::perception::LabelsReader labels_reader;
    if (!labels_reader.init(label_file_name)) {
        std::cerr << "Open Label File:" << label_file_name << " failed." << std::endl;
        return -1;
    }
    
    std::vector<boost::shared_ptr<std::ofstream> > oss(3);
    for (int i = 0; i < oss.size(); i++) {
        oss[i] = boost::shared_ptr<std::ofstream>(new std::ofstream);
        std::stringstream file_name;
        file_name << "./features-" << ('X'+i);
        oss[i]->open(file_name.str().c_str(), std::ios_base::out);
        if (!(*oss[i])) {
            std::cerr << "Open Output file:" << file_name << std::endl;
            return -1;
        }
    }

    adu::perception::LabelsReader::Iter iter = labels_reader.begin();
    while(iter != labels_reader.end()) {
        const std::string& pcd_file_name = iter->first;
        const Label::Ptr label = iter->second;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = read_pcd(data_root + pcd_file_name);        
        if (!point_cloud) { return -1; }

        //draw_point_cloud_and_bounding_box(point_cloud, label);
        draw_hog_features(oss, point_cloud, label); 
        iter++;
    }
    for (int i = 0; i < oss.size(); i++) {
        oss[i]->close();
    }
    return 0;
}
