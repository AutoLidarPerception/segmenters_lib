/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_BASE_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_BASE_SEGMENTER_HPP_

#include <ros/ros.h>
#include <string>
#include <vector>

#include "common/types/type.h"

namespace autosense {
namespace segmenter {

class BaseSegmenter {
 public:
    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud &cloud_in,
        std::vector<PointICloudPtr> &cloud_clusters) = 0;  // NOLINT

    virtual std::string name() const = 0;
};  // BaseSegmenter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_BASE_SEGMENTER_HPP_
