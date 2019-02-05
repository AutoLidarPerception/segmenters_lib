/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/region_euclidean_segmenter.hpp"

#include <ros/ros.h>

#include "common/bounding_box.hpp"
#include "common/geometry.hpp"  // common::geometry::calcCylinderDist
#include "common/time.hpp"      // common::Clock
#include "object_builders/object_builder_manager.hpp"  // object_builder::createObjectBuilder()

namespace autosense {
namespace segmenter {

RegionEuclideanSegmenter::RegionEuclideanSegmenter() {}

RegionEuclideanSegmenter::RegionEuclideanSegmenter(
    const SegmenterParams& params)
    : params_(params), kd_tree_(new pcl::search::KdTree<PointI>) {
    if (params_.rec_region_size > region_upper_bound_) {
        params_.rec_region_size = region_upper_bound_;
        ROS_WARN_STREAM("Region size reached upper bound ("
                        << region_upper_bound_ << "), use it instead.");
    }

    euclidean_cluster_extractor_.setMinClusterSize(
        params_.rec_min_cluster_size);
    euclidean_cluster_extractor_.setMaxClusterSize(
        params_.rec_max_cluster_size);
    euclidean_cluster_extractor_.setSearchMethod(kd_tree_);

    if (params_.rec_use_region_merge) {
        object_builder_ = object_builder::createObjectBuilder();
    }
}

RegionEuclideanSegmenter::~RegionEuclideanSegmenter() {}

void RegionEuclideanSegmenter::segment(
    const PointICloud &cloud_in, std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty non-ground for segmentation, do nonthing.");
        return;
    }

    // Clear segments.
    cloud_clusters.clear();

    common::Clock clock;
    ROS_INFO("Starting Region Euclidean segmentation.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    /**
     * @brief Adaptive distance threshold Euclidean Cluster Extraction
     *
     * @note distance threshold is challenging in practice
     *  If too small, a single object could be split into multiple clusters
     *  If too high, multiple objects could be merged into one cluster
     * @note the shape formed by laser beams irradiated on the human body can be
     *  very different, depending on the distance of the person from the sensor
     *
     * ==> Adaptive method to determine distance according to different scan
     * ranges
     * <?> distance threshold d = 2*r*tan(Θ/2)
     *   r is the scan range of the 3D LiDAR TODO==>zone_[nested_regions_]
     *   Θ is the fixed vertical angular resolution
     *   d is the vertical distance between two adjacent laser scans
     * <1> divide the space into nested circular regions centred at the sensor
     * <2> different distance thresholds are applied
     */
    boost::array<std::vector<int>, region_upper_bound_> nested_regions;

    // ROS_WARN_STREAM("nested_regions.size() = " << nested_regions.size());
    // <1> divide the space into nested circular regions centred at the sensor
    for (size_t pt = 0u; pt < cloud->size(); ++pt) {
        // Sphere distance
        // float dist = common::calcSphereDist(cloud_roi->points[i]);
        // Cylinder distance
        float dist = common::geometry::calcCylinderDist(cloud->points[pt]);

        // assign specific belonging region for every point
        float rRange = 0.0;
        for (size_t region = 0u; region < params_.rec_region_size; ++region) {
            if (dist > rRange &&
                dist <= (rRange + params_.rec_region_sizes[region])) {
                nested_regions[region].push_back(pt);
                break;
            }
            rRange += params_.rec_region_sizes[region];
        }
    }

// #define _SHOW_CLUSTERING_REGION_
#ifdef _SHOW_CLUSTERING_REGION_
    // circular regions one by one
    for (size_t region = 0u; region < params_.rec_region_size; ++region) {
        PointICloudPtr cluster(new PointICloud);
        pcl::copyPointCloud(*cloud, nested_regions[region], *cluster);
        cloud_clusters->push_back(cluster);
    }
#else
    // variables for clusters merge adjacent regions
    std::vector<std::vector<int> > clusters_region_map(params_.rec_region_size);
    uint32_t clusterId = 0u;
    // std::vector<float> distMins;
    // std::vector<float > distMaxs;

    // circular regions one by one
    double tolerance = params_.rec_region_initial_tolerance;
    for (size_t region = 0u; region < params_.rec_region_size; ++region) {
        // avoid useless Euclidean Cluster Extraction
        if (nested_regions[region].size() > params_.rec_min_cluster_size) {
            // <2> different distance thresholds are applied
            boost::shared_ptr<std::vector<int> > region_indices(
                new std::vector<int>(nested_regions[region]));

            std::vector<pcl::PointIndices> clusters_indices;
            euclidean_cluster_extractor_.setClusterTolerance(tolerance);
            euclidean_cluster_extractor_.setInputCloud(cloud);
            euclidean_cluster_extractor_.setIndices(region_indices);
            euclidean_cluster_extractor_.extract(clusters_indices);

            for (std::vector<pcl::PointIndices>::const_iterator citer =
                     clusters_indices.begin();
                 citer != clusters_indices.end(); citer++) {
                PointICloudPtr cluster(new PointICloud);
                pcl::copyPointCloud(*cloud, *citer, *cluster);

                // TODO(gary): calculate segment's min/max cyclinder distance
                /*std::vector<int>::const_iterator piter =
                citer->indices.begin();
                float distMin = common::calcCylinderDist(cloud->points[*piter]);
                float distMax = common::calcCylinderDist(cloud->points[*piter]);
                for ( ; piter != citer->indices.end(); ++piter) {
                    cluster->points.push_back(cloud->points[*piter]);
                    float dist =
                common::calcCylinderDist(cloud->points[*piter]);
                    if (dist < distMin) {
                        distMin = dist;
                    }
                    if (dist > distMax) {
                        distMax = dist;
                    }
                }*/

                cloud_clusters.push_back(cluster);
                if (params_.rec_use_region_merge) {
                    clusters_region_map[region].push_back(clusterId++);
                }

                // TODO(gary): record corresponding region index & min/max dist
                /*distMins.push_back(distMin);
                distMaxs.push_back(distMax);*/
            }
        }
        // different distance thresholds are applied
        tolerance += params_.rec_region_delta_tolerance;
    }

    /*
     * TODO(gary): how to merge two clusters A,B cut by circular regions
     *  A.B in the same circular region: no need to merge
     *  A.B not in the same circular region: Ra_min Ra_max Rb_min Rb_max
     *      A inside, B outside: Ra_max<Rb_min
     *      B inside, A outside: Rb_max<Ra_min
     *      threshold: min max bias less than 1/10 of the width
     */
    if (params_.rec_use_region_merge) {
        bool had_merged = false;

        // build clusters' 3D OBB
        std::vector<ObjectPtr> clusters;
        object_builder_->build(cloud_clusters, &clusters);

        for (size_t region = 1u; region < params_.rec_region_size; ++region) {
            // clusters in current outside region
            // #clusters_region_map[regionIdx][i]
            for (size_t out_idx = 0u;
                 out_idx < clusters_region_map[region].size(); ++out_idx) {
                int clusterId_outside = clusters_region_map[region][out_idx];
                common::bbox::GroundBox gbox_out;
                common::bbox::toGroundBox(clusters[clusterId_outside],
                                          &gbox_out);

                // clusters in previous inside region
                // #clusters_region_map[regionIdx-1][j]
                for (size_t in_idx = 0;
                     in_idx < clusters_region_map[region - 1].size();
                     ++in_idx) {
                    int clusterId_inside =
                        clusters_region_map[region - 1][in_idx];
                    // one inside should merged into only one outside
                    if (cloud_clusters[clusterId_inside]->empty()) {
                        continue;
                    }
                    common::bbox::GroundBox gbox_in;
                    common::bbox::toGroundBox(clusters[clusterId_inside],
                                              &gbox_in);

                    if (common::bbox::groundBoxIoU(gbox_in, gbox_out) >
                        params_.rec_region_merge_tolerance) {
                        *cloud_clusters[clusterId_outside] +=
                            *cloud_clusters[clusterId_inside];
                        // ROS_INFO_STREAM("Merged Cluster #" <<
                        // clusterId_inside << " to #"
                        // << clusterId_outside << ".");
                        // reconstruct merged cluster
                        *clusters[clusterId_outside]->cloud +=
                            *cloud_clusters[clusterId_inside];
                        object_builder_->build(clusters[clusterId_outside]);

                        cloud_clusters[clusterId_inside]->clear();
                        had_merged = true;
                    }
                }
            }
        }

        // remove all merged clusters at once
        if (had_merged) {
            std::vector<PointICloudPtr>::iterator iter = cloud_clusters.begin();
            for (; iter != cloud_clusters.end();) {
                if ((*iter)->empty()) {
                    iter = cloud_clusters.erase(iter);
                } else {
                    ++iter;
                }
            }
        }
    }
#endif

    ROS_INFO_STREAM("RegionEuclideanSegmenter Segmentation complete. Took "
                    << clock.takeRealTime() << "ms.");
}

}  // namespace segmenter
}  // namespace autosense
