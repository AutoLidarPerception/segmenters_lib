/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_REGION_EUCLIDEAN_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_REGION_EUCLIDEAN_SEGMENTER_HPP_

#include <pcl/search/kdtree.h>                  // pcl::search::KdTree
#include <pcl/segmentation/extract_clusters.h>  // pcl::EuclideanClusterExtraction
#include <string>
#include <vector>

#include "object_builders/base_object_builder.hpp"  // object_builder::BaseObjectBuilder
#include "segmenters/base_segmenter.hpp"

namespace autosense {
namespace segmenter {

class RegionEuclideanSegmenter : public BaseSegmenter {
 public:
    RegionEuclideanSegmenter();

    explicit RegionEuclideanSegmenter(const SegmenterParams &params);

    ~RegionEuclideanSegmenter();

    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud &cloud_in,
        std::vector<PointICloudPtr> &cloud_clusters);  // NOLINT

    virtual std::string name() const { return "RegionEuclideanSegmenter"; }

 private:
    SegmenterParams params_;

    // nested circular regions centred at the sensor with width
    // nested_regions_[i](Ri −Ri−1)
    static const int region_upper_bound_ = 14;
    // int nested_regions_[region_upper_bound_] = {2, 3, 3, 3, 3, 3, 3, 2, 3, 3,
    // 3, 3, 3, 3};

    pcl::search::KdTree<PointI>::Ptr kd_tree_;
    pcl::EuclideanClusterExtraction<PointI> euclidean_cluster_extractor_;

    // used for clusters merged between regions
    boost::shared_ptr<object_builder::BaseObjectBuilder> object_builder_;
};  // class RegionEuclideanSegmenter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_REGION_EUCLIDEAN_SEGMENTER_HPP_
