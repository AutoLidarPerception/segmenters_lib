/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_EUCLIDEAN_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_EUCLIDEAN_SEGMENTER_HPP_

#include <pcl/search/kdtree.h>                  // pcl::search::KdTree
#include <pcl/segmentation/extract_clusters.h>  // pcl::EuclideanClusterExtraction
#include <string>
#include <vector>
#include "segmenters/base_segmenter.hpp"

namespace autosense {
namespace segmenter {

class EuclideanSegmenter : public BaseSegmenter {
 public:
    EuclideanSegmenter();

    explicit EuclideanSegmenter(const SegmenterParams& params);

    ~EuclideanSegmenter();

    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud& cloud_in,
        std::vector<PointICloudPtr>& cloud_clusters);  // NOLINT

    virtual std::string name() const { return "EuclideanSegmenter"; }

 private:
    SegmenterParams params_;

    pcl::search::KdTree<PointI>::Ptr kd_tree_;
    pcl::EuclideanClusterExtraction<PointI> euclidean_cluster_extractor_;
};  // class EuclideanSegmenter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_EUCLIDEAN_SEGMENTER_HPP_
