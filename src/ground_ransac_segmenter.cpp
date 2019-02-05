/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/ground_ransac_segmenter.hpp"

#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>              // pcl::ModelCoefficients
#include <pcl/filters/extract_indices.h>        // pcl::ExtractIndices
#include <pcl/io/io.h>                          // pcl::copyPointCloud
#include <pcl/sample_consensus/method_types.h>  // pcl::SAC_RANSAC
#include <pcl/sample_consensus/model_types.h>   // pcl::SACMODEL_PLANE

#include "common/time.hpp"  // common::Clock

namespace autosense {
namespace segmenter {

GroundRANSACSegmenter::GroundRANSACSegmenter() {}

GroundRANSACSegmenter::GroundRANSACSegmenter(const SegmenterParams& params)
    : params_(params) {
    // Optional
    sac_estimator_.setOptimizeCoefficients(true);
    // Fitting models like planes, cylinders...
    sac_estimator_.setModelType(pcl::SACMODEL_PLANE);
    // SAmple Consensus (SAC_) methods like RANSAC, LMEDS (Least Median of
    // Squares)
    sac_estimator_.setMethodType(pcl::SAC_RANSAC);
    // Distance close to the model to be considered as an inlier, default 0
    sac_estimator_.setDistanceThreshold(params_.sac_distance_threshold);
    // Default 50
    // sac_estimator_.setMaxIterations(params_.sac_max_iteration);
    // Default 0.99
    // sac_estimator_.setProbability(params_.sac_probability);
}

GroundRANSACSegmenter::~GroundRANSACSegmenter() {}

/**
 * @breif Ground Removal based-on Voxel Grid Filter
 * @note Cascade RANSAC is invalid...
 * @note Multi-segment RANSAC
 */
const int segment_number = 3;
int segments[segment_number] = {20, 30, 40};

void GroundRANSACSegmenter::segment(
    const PointICloud &cloud_in, std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty ground for segmentation, do nonthing.");
        return;
    }
    // Clear segments.
    cloud_clusters.clear();
    bool ground_removed = false;

    common::Clock clock;
    ROS_INFO("Starting Ground RANSAC segmentation.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);

    sac_estimator_.setInputCloud(cloud);
    sac_estimator_.segment(*ground_indices, *coefficients);

    if (ground_indices->indices.size() > 0) {
        pcl::ExtractIndices<PointI> indiceExtractor;
        indiceExtractor.setInputCloud(cloud);
        indiceExtractor.setIndices(ground_indices);

        // extract ground points
        indiceExtractor.setNegative(false);
        indiceExtractor.filter(*cloud_ground);

        // extract non-ground points
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*cloud_nonground);

        ground_removed = true;
    }

    cloud_clusters.push_back(cloud_ground);
    cloud_clusters.push_back(cloud_nonground);

#ifdef X_axis
    boost::array<std::vector<int>, segment_number> nested_segments;
    for (size_t pt = 0u; pt < cloud->points.size(); ++pt) {
        for (size_t seg = 0u; seg < segment_number; ++seg) {
            if (cloud->points[pt].x < segments[seg]) {
                nested_segments[seg].push_back(pt);
            }
        }
    }
    for (size_t seg = 0u; seg < segment_number; ++seg) {
        pcl::IndicesPtr indices(new std::vector<int>(nested_segments[seg]));
        sac_segmenter_.setInputCloud(cloud);
        sac_segmenter_.setIndices(indices);

        sac_segmenter_.segment(*ground_indices, *coefficients);

        if (ground_indices->indices.size() > 0) {
            pcl::ExtractIndices<Point> indiceExtractor;
            indiceExtractor.setInputCloud(cloud);
            indiceExtractor.setIndices(ground_indices);
            // extract ground points
            indiceExtractor.setNegative(false);
            indiceExtractor.filter(*cloud_ground);
            // extract non-ground points
            indiceExtractor.setNegative(true);
            indiceExtractor.filter(*cloud_nonground);

            *pc_ground += *cloud_ground;
            *pc_nonground += *cloud_nonground;
            ground_removed = true;
        }
    }
#endif

#ifdef DivideBySort
    std::vector<Point> points(cloud->points.begin(), cloud->points.end());
    std::sort(points.begin(), points.end(), common::sortByAxisXAsc<Point>);

    // +3 makes sure the last segment contains last point
    int segment_size = points.size() / ground_removal_segment_size_ + 3;
    int segment_id = 1;
    cloud->clear();
    for (size_t pt = 0u; pt < points.size(); ++pt) {
        cloud->push_back(points[pt]);
        if (pt == segment_size * segment_id || pt == (points.size() - 1)) {
            seg.setInputCloud(cloud);
            seg.segment(*ground_indices, *coefficients);

            if (ground_indices->indices.size() > 0) {
                pcl::ExtractIndices<Point> indiceExtractor;
                indiceExtractor.setInputCloud(cloud);
                indiceExtractor.setIndices(ground_indices);
                // extract ground points
                indiceExtractor.setNegative(false);
                indiceExtractor.filter(*cloud_ground);
                // extract non-ground points
                indiceExtractor.setNegative(true);
                indiceExtractor.filter(*cloud_nonground);

                *pc_ground += *cloud_ground;
                *pc_nonground += *cloud_nonground;
                ground_removed = true;
            }

            segment_id++;
            cloud->clear();
        }
    }
#endif

#ifdef CascadeRANSAC
    seg.setInputCloud(cloud);
    seg.segment(*ground_indices, *coefficients);

    if (ground_indices->indices.size() > 0) {
        pcl::ExtractIndices<Point> indiceExtractor;
        indiceExtractor.setInputCloud(cloud);
        indiceExtractor.setIndices(ground_indices);
        // extract ground points
        indiceExtractor.setNegative(false);
        indiceExtractor.filter(*cloud_ground);
        *pc_ground = *cloud_ground;

        // extract non-ground points
        // false: extract the inliers
        // true:  filter the inliers
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*cloud_nonground);

        // Try second ...
        seg.setInputCloud(cloud_nonground);
        seg.segment(*ground_indices, *coefficients);
        if (ground_indices->indices.size() > ground_removal_2nd_min_size_) {
            indiceExtractor.setInputCloud(cloud_nonground);
            indiceExtractor.setIndices(ground_indices);

            indiceExtractor.setNegative(true);
            indiceExtractor.filter(*pc_nonground);

            indiceExtractor.setNegative(false);
            indiceExtractor.filter(*cloud_ground);
            *pc_ground += *cloud_ground;
        }

        ROS_INFO_STREAM("Ground removed. Took " << clock.takeRealTime()
                                                << " ms.");

        return true;
    } else {
        return false;
    }
#endif

    ROS_INFO_STREAM("Segmentation complete. Took " << clock.takeRealTime()
                                                   << "ms.");
}

}  // namespace segmenter
}  // namespace autosense
