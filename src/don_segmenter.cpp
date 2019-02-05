/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/don_segmenter.hpp"
#include <ros/ros.h>
#include "common/time.hpp"

namespace autosense {
namespace segmenter {

DoNSegmenter::DoNSegmenter() {}

DoNSegmenter::DoNSegmenter(const SegmenterParams &params)
    : kd_tree_(new pcl::search::KdTree<PointI>),
      params_(params),
      kd_tree_seg_(new pcl::search::KdTree<PointN>) {
    normal_estimator_omp_.setSearchMethod(kd_tree_);

    /// @note Ensure that the normals are all pointed in the same direction.
    normal_estimator_omp_.setViewPoint(std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max());

    // Build the condition for filtering
    pcl::ConditionOr<PointN>::Ptr range_cond(new pcl::ConditionOr<PointN>());
    range_cond->addComparison(
        pcl::FieldComparison<PointN>::ConstPtr(new pcl::FieldComparison<PointN>(
            // Greater Than
            // "curvature", pcl::ComparisonOps::GT,
            // params_.don_segmenter_range_threshold
            "curvature", pcl::ComparisonOps::GT,
            params_.don_segmenter_range_threshold)));
    range_cond_filter_.setCondition(range_cond);

    ec_extractor_.setMinClusterSize(params_.don_segmenter_ec_min_size);
    ec_extractor_.setMaxClusterSize(params_.don_segmenter_ec_max_size);
    ec_extractor_.setClusterTolerance(params_.don_segmenter_ec_tolerance);
}

DoNSegmenter::~DoNSegmenter() {
    kd_tree_.reset();
    kd_tree_seg_.reset();
}

void DoNSegmenter::segment(const PointICloud &cloud_in,
                           std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty non-ground for segmentation, do nonthing.");
        return;
    }
    // Clear segments.
    cloud_clusters.clear();

    common::Clock clock;
    ROS_INFO("Starting Difference of Normals segmentation.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    // Set the input pointcloud for the search tree
    kd_tree_->setInputCloud(cloud);

    // Large/Small Radius Normal Estimation
    normal_estimator_omp_.setInputCloud(cloud);
    normal_estimator_omp_.setSearchMethod(kd_tree_);

    // calculate normals with the small scale
    PointNCloudPtr normal_cloud_small(new PointNCloud);
    normal_estimator_omp_.setRadiusSearch(params_.don_segmenter_small_scale);
    normal_estimator_omp_.compute(*normal_cloud_small);
    // calculate normals with the large scale
    PointNCloudPtr normal_cloud_large(new PointNCloud);
    normal_estimator_omp_.setRadiusSearch(params_.don_segmenter_large_scale);
    normal_estimator_omp_.compute(*normal_cloud_large);

    // Difference of Normals Feature Calculation
    // create output cloud for DoN results
    PointNCloudPtr don_cloud(new PointNCloud);
    pcl::copyPointCloud<PointI, PointN>(*cloud, *don_cloud);

    DoN_estimator_.setInputCloud(cloud);
    DoN_estimator_.setNormalScaleSmall(normal_cloud_small);
    DoN_estimator_.setNormalScaleLarge(normal_cloud_large);
    if (!DoN_estimator_.initCompute()) {
        ROS_ERROR("Could not intialize DoN feature estimator.");
        return;
    }
    // compute DoN
    DoN_estimator_.computeFeature(*don_cloud);

    // Difference of Normals Based Filtering
    PointNCloudPtr don_cloud_filtered(new PointNCloud);
    range_cond_filter_.setInputCloud(don_cloud);
    range_cond_filter_.filter(*don_cloud_filtered);

    // Clustering the Results
    std::vector<pcl::PointIndices> clusters_indices;
    kd_tree_seg_->setInputCloud(don_cloud_filtered);
    ec_extractor_.setSearchMethod(kd_tree_seg_);
    ec_extractor_.setInputCloud(don_cloud_filtered);
    ec_extractor_.extract(clusters_indices);
    std::vector<pcl::PointIndices>::const_iterator iter =
        clusters_indices.begin();
    for (; iter != clusters_indices.end(); ++iter) {
        PointICloudPtr cluster(new PointICloud);
        pcl::copyPointCloud<PointN, PointI>(*don_cloud_filtered, *iter,
                                            *cluster);

        cloud_clusters.push_back(cluster);
    }

    ROS_INFO_STREAM("Segmentation complete. Took " << clock.takeRealTime()
                                                   << "ms.");
}

}  // namespace segmenter
}  // namespace autosense
