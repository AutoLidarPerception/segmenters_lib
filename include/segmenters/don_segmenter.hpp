/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_DON_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_DON_SEGMENTER_HPP_

#include <pcl/features/don.h>            // pcl::DifferenceOfNormalsEstimation
#include <pcl/features/normal_3d_omp.h>  // pcl::NormalEstimationOMP
#include <pcl/filters/conditional_removal.h>    // pcl::ConditionalRemoval
#include <pcl/search/kdtree.h>                  // pcl::search::KdTree
#include <pcl/segmentation/extract_clusters.h>  // pcl::EuclideanClusterExtraction
#include <string>
#include <vector>

#include "segmenters/base_segmenter.hpp"

namespace autosense {
namespace segmenter {

class DoNSegmenter : public BaseSegmenter {
 public:
    DoNSegmenter();

    explicit DoNSegmenter(const SegmenterParams& params);

    ~DoNSegmenter();

    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud &cloud_in,
        std::vector<PointICloudPtr> &cloud_clusters);  // NOLINT

    virtual std::string name() const { return "DoNSegmenter"; }

 private:
    SegmenterParams params_;

    /**
     * @breif For our later calls to calculate normals, we need to create a
     * search
     * tree.
     * @note
     *  Organized data (i.e. a depth image): OrganizedNeighbor search tree
     *  Unorganized data (i.e. LiDAR scans): KDTree
     */
    pcl::search::KdTree<PointI>::Ptr kd_tree_;
    /**
     * @brief makes use of OpenMP to use many threads to calculate the normal
     * @note estimating the normals, also the bottleneck computationally
     *  standard single-threaded class: pcl::NormalEstimation
     *  GPU accelerated class: pcl::gpu::NormalEstimation
     */
    pcl::NormalEstimationOMP<PointI, PointN> normal_estimator_omp_;
    pcl::DifferenceOfNormalsEstimation<PointI, PointN, PointN> DoN_estimator_;
    /**
     * @note Build the filter
     */
    pcl::ConditionalRemoval<PointN> range_cond_filter_;

    /**
     * @brief
     */
    pcl::search::KdTree<PointN>::Ptr kd_tree_seg_;
    pcl::EuclideanClusterExtraction<PointN> ec_extractor_;
};  // class DoNSegmenter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_DON_SEGMENTER_HPP_
