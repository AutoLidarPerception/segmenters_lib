/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/euclidean_segmenter.hpp"

#include <pcl/filters/extract_indices.h>  // pcl::ExtractIndices
#include <ros/ros.h>

#include "common/time.hpp"  // common::Clock

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl/point_cloud.h> /* pcl::PointCloud */
#include <pcl/point_types.h> /* pcl::PointXYZ */
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl/io/io.h>                    // pcl::copyPointCloud
#include <pcl/common/projection_matrix.h>

namespace autosense {
namespace segmenter {

EuclideanSegmenter::EuclideanSegmenter() {}

EuclideanSegmenter::EuclideanSegmenter(const SegmenterParams& params)
    : params_(params), kd_tree_(new pcl::search::KdTree<PointI>) {
    euclidean_cluster_extractor_.setSearchMethod(kd_tree_);
    euclidean_cluster_extractor_.setMinClusterSize(params_.ec_min_cluster_size);
    euclidean_cluster_extractor_.setMaxClusterSize(params_.ec_max_cluster_size);
    euclidean_cluster_extractor_.setClusterTolerance(params_.ec_tolerance);
}

EuclideanSegmenter::~EuclideanSegmenter() {}

void EuclideanSegmenter::segment(const PointICloud &cloud_in,
                                 std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty non-ground for segmentation, do nonthing.");
        return;
    }
    // Clear segments.
    cloud_clusters.clear();

    common::Clock clock;
    ROS_INFO("Starting Euclidean segmentation.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;
    //--------------------------------
    //Here I start my magic
    PointICloudPtr cloud_filtered(new PointICloud);
    cloud_filtered->clear();
    const PointICloud& project_cloud = cloud_in;

    pcl::PointIndices::Ptr filtered_indices(new pcl::PointIndices);
    filtered_indices->indices.clear();

    for (size_t pt = 0u; pt < project_cloud.points.size(); ++pt){
        project_cloud.points[pt].z == 0.0;
        filtered_indices->indices.push_back(pt);
    }
    pcl::copyPointCloud(project_cloud, *filtered_indices, *cloud_filtered);
    //Here my magic end
    //--------------------------------
    std::vector<pcl::PointIndices> cluster_indices;

    // extract clusters
    euclidean_cluster_extractor_.setInputCloud(cloud);
    euclidean_cluster_extractor_.extract(cluster_indices);

    if (cluster_indices.size() > 0) {
        for (size_t cluster_idx = 0u; cluster_idx < cluster_indices.size();
             ++cluster_idx) {
            PointICloudPtr cluster_cloud(new PointICloud);
            pcl::copyPointCloud(*cloud, cluster_indices[cluster_idx],
                                *cluster_cloud);
            cloud_clusters.push_back(cluster_cloud);
        }
    }

    ROS_INFO_STREAM("Segmentation complete. Took " << clock.takeRealTime()
                                                   << "ms.");
}

}  // namespace segmenter
}  // namespace autosense
