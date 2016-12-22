// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/22 16:29:52
// @file label_reader.h
// @brief 
#ifndef ADU_PERCEPTION_LABEL_READER_H
#define ADU_PERCEPTION_LABEL_READER_H

#include "types.h"

namespace adu {
namespace perception {

class LabelsReader {
public:
    bool init(const std::string& file_name);

    const std::vector<Label::Ptr>& get(const std::string& pcd_file_name) const {
        return _labels[pcd_file_name];
    }
private:
    std::string _labels_file_name;
    mutable std::unordered_map<std::string, std::vector<Label::Ptr>> _labels;
};

} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_LABEL_READER_H
// 

