// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/29 20:29:14
// @file features.cpp
// @brief 
// 
//
#include "features.h" 

namespace adu {
namespace perception {

FeatureExtractor::FeatureExtractor(bool save_to_file, 
                                   const std::string& file_prefix) : 
    _save_to_file(save_to_file), 
    _features_num(4) {
    if (!_save_to_file) return;    

    _oss.resize(_features_num); 
    for (int i = 0; i < _oss.size(); i++) {
        _oss[i] = boost::shared_ptr<std::ofstream>(new std::ofstream);
        std::stringstream file_name;
        file_name << file_prefix << "-" << i;
        _oss[i]->open(file_name.str().c_str(), std::ios_base::out);
        if (!(*_oss[i])) {
            std::cerr << "Open Output file:" << file_name << std::endl;
            throw std::runtime_error("open file failed");
        }
    }
}

FeatureExtractor::~FeatureExtractor() {
    for (int i = 0; i < _oss.size(); i++) {
        _oss[i]->close();
    }
}

bool FeatureExtractor::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                               const Label::Ptr label) {
    
    for (size_t i = 0; i < label->boxes.size(); i++) {
        // Get Object Point
        const Box::Ptr& box = label->boxes[i];
        BOOST_AUTO(object_indices, adu::perception::BoxFilter::filter(point_cloud, *box));
        BOOST_AUTO(object, adu::perception::BoxFilter::filter(point_cloud, object_indices));
        if (object->points.size() < 10)  continue; 

        // Compute Features.
        std::vector<std::vector<float> > features(_features_num);
        if (!(_hog_feature_extractor.compute(object, &features) &&
              _bounding_box_feature_extractor.compute(object, &features[3]))) {
            return false; 
        }
#ifdef SHOW_DETAIL
        std::cout << "Box:" << box->type_str() << " height:"<< features[3][0] << " width:" << features[3][1] << " height:" << features[3][2] << std::endl;
#endif

        // Save File
        if (!_save_to_file) continue;
#pragma omp parallel for
        for (size_t i = 0; i < _oss.size(); i++) {
            adu::perception::serialize_features(*_oss[i], box->id_str(), box->type_str(), features[i]);
        }
    }
    return true;
}

void serialize_features(std::ostream& os, const std::string& id ,const std::string& type, const std::vector<float>& features) {
    if (features.empty()) {
        std::cout << "id:" << id << " has no features." << std::endl;
    }
    os << id <<"\t";
    os << type << "\t";
    for (int i = 0; i < features.size(); i++) {
        os << features[i];
        if (i != features.size() - 1) {
            os << "\t";
        } else {
            os << std::endl;
        }
    }
}

} // namespace perception
} // namespace adu
