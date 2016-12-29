// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/29 20:25:14
// @file features.h
// @brief 
#ifndef ADU_PERCEPTION_PCD_BOUING_BOX_FEATURES_H
#define ADU_PERCEPTION_PCD_BOUING_BOX_FEATURES_H

#include <iostream>

#include "bounding_box_feature.h"
#include "hog_feature.h"

namespace adu {
namespace perception {

class FeatureExtractor {
public:
    FeatureExtractor(bool save_to_file, const std::string& file_prefix);
    ~FeatureExtractor();
    bool extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr object,
                 const Label::Ptr label);

private:
    BoundingBoxFeature _bounding_box_feature_extractor;
    HogFeature         _hog_feature_extractor; 

    int                _features_num;
    bool               _save_to_file;
    std::vector<boost::shared_ptr<std::ofstream> > _oss;
};

void serialize_features(std::ostream& os, 
                        const std::string& id, 
                        const std::string& type, 
                        const std::vector<float>& features);
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_PCD_BOUING_BOX_FEATURES_H

