/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/region_growing_segmenter.hpp"

#include "common/time.hpp"

namespace autosense {
namespace segmenter {

RegionGrowingSegmenter::RegionGrowingSegmenter() {}

RegionGrowingSegmenter::RegionGrowingSegmenter(const SegmenterParams &params)
    : kd_tree_(new pcl::search::KdTree<PointI>), params_(params) {
    normal_estimator_omp_.setSearchMethod(kd_tree_);

    /// @note Ensure that the normals are all pointed in the same direction.
    normal_estimator_omp_.setViewPoint(std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max());
    if (params_.rg_knn_for_normals <= 0) {
        if (params_.rg_radius_for_normals <= 0.0) {
            ROS_ERROR(
                "Wrong parameters for Region Growing normals estimation.");
        } else {
            normal_estimator_omp_.setRadiusSearch(
                params_.rg_radius_for_normals);
            ROS_INFO_STREAM("Region Growing normals estimation based on radius "
                            << params_.rg_radius_for_normals << ".");
        }
    } else {
        normal_estimator_omp_.setKSearch(params_.rg_knn_for_normals);
        ROS_INFO_STREAM("Region Growing normals estimation based on knn "
                        << params_.rg_knn_for_normals << ".");
    }

    region_growing_estimator_.setMinClusterSize(params_.rg_min_cluster_size);
    region_growing_estimator_.setMaxClusterSize(params_.rg_max_cluster_size);
    region_growing_estimator_.setSearchMethod(kd_tree_);
    region_growing_estimator_.setNumberOfNeighbours(params_.rg_knn_for_growing);

    region_growing_estimator_.setSmoothModeFlag(true);
    region_growing_estimator_.setSmoothnessThreshold(
        params_.rg_smoothness_threshold_deg / 180.0 * M_PI);

    region_growing_estimator_.setCurvatureTestFlag(true);
    region_growing_estimator_.setCurvatureThreshold(
        params_.rg_curvature_threshold);

    region_growing_estimator_.setResidualTestFlag(false);
}

RegionGrowingSegmenter::~RegionGrowingSegmenter() {
    kd_tree_.reset();
    // range_cond_filter_.reset();
}

void RegionGrowingSegmenter::segment(
    const PointICloud &cloud_in, std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty non-ground for segmentation, do nonthing.");
        return;
    }
    // Clear segments.
    cloud_clusters.clear();

    common::Clock clock;
    ROS_INFO("Starting region growing segmentation.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    // Normal Estimation
    NormalCloudPtr normals(new NormalCloud);
    normal_estimator_omp_.setInputCloud(cloud);
    normal_estimator_omp_.compute(*normals);
    ROS_INFO_STREAM("Normals are computed. Took " << clock.takeRealTime()
                                                  << "ms.");

    // Remove points with high curvature.
    clock.start();
    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);
    pcl::PointIndices::Ptr care_indices(new pcl::PointIndices);
    for (size_t i = 0u; i < normals->size(); ++i) {
        if (normals->points[i].curvature < params_.rg_curvature_threshold) {
            care_indices->indices.push_back(i);
        }
        /*if (cloud_normals->points[i].normal_z <
        params_.rg_curvature_threshold) {
            ground_indices->indices.push_back(i);
        }*/
    }

    region_growing_estimator_.setInputCloud(cloud);
    region_growing_estimator_.setInputNormals(normals);
    region_growing_estimator_.setIndices(care_indices);
    std::vector<pcl::PointIndices> clusters_indices;
    region_growing_estimator_.extract(clusters_indices);
    std::vector<pcl::PointIndices>::const_iterator iter =
        clusters_indices.begin();
    for (; iter != clusters_indices.end(); ++iter) {
        PointICloudPtr cluster(new PointICloud);
        pcl::copyPointCloud(*cloud, *iter, *cluster);

        cloud_clusters.push_back(cluster);
    }

    /*if (care_indices->indices.size()) {
        pcl::ExtractIndices<Point> indiceExtractor;
        indiceExtractor.setInputCloud(cloud);
        indiceExtractor.setIndices(care_indices);
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*cloud_nonground);
        pc_clusters.push_back(cloud_nonground);

        indiceExtractor.setNegative(false);
        indiceExtractor.filter(*cloud_ground);
        pc_clusters.push_back(cloud_ground);
    }*/

    ROS_INFO_STREAM("Segmentation complete. Took " << clock.takeRealTime()
                                                   << "ms.");
}

}  // namespace segmenter
}  // namespace autosense
