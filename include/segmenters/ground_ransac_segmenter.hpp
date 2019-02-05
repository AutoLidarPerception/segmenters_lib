/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_RANSAC_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_RANSAC_SEGMENTER_HPP_

#include <pcl/segmentation/sac_segmentation.h>  // pcl::SACSegmentation
#include <string>
#include <vector>

#include "common/types/type.h"
#include "segmenters/base_segmenter.hpp"

namespace autosense {
namespace segmenter {

class GroundRANSACSegmenter : public BaseSegmenter {
 public:
    GroundRANSACSegmenter();

    explicit GroundRANSACSegmenter(const SegmenterParams &params);

    ~GroundRANSACSegmenter();

    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud &cloud_in,
        std::vector<PointICloudPtr> &cloud_clusters);  // NOLINT

    virtual std::string name() const { return "GroundRANSACSegmenter"; }

 private:
    SegmenterParams params_;

    pcl::SACSegmentation<PointI> sac_estimator_;
};  // class GroundRANSACSegmenter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_RANSAC_SEGMENTER_HPP_
