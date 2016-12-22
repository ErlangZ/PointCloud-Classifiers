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

    const std::vector<Label>& data() const {
        return _labels;
    }
private:
    std::string _file_name;
    std::vector<Label> _labels;
};

} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_LABEL_READER_H
// 

