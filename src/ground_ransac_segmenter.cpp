#include "segmenters/ground_ransac_segmenter.hpp"

#include <ros/ros.h>

#include <pcl/sample_consensus/model_types.h>       /* pcl::SACMODEL_PLANE */
#include <pcl/sample_consensus/method_types.h>      /* pcl::SAC_RANSAC */
#include <pcl/ModelCoefficients.h>                  /* pcl::ModelCoefficients */
#include <pcl/filters/extract_indices.h>            /* pcl::ExtractIndices */
#include <pcl/io/io.h>                              /* pcl::copyPointCloud */

#include "common/time.hpp"                          /* common::Clock */

namespace segmenter {
    GroundRANSACSegmenter::GroundRANSACSegmenter()
    {}

    GroundRANSACSegmenter::GroundRANSACSegmenter(const SegmenterParams& params) :
            params_(params)
    {
        // Optional
        sac_estimator_.setOptimizeCoefficients(true);
        // Fitting models like planes, cylinders...
        sac_estimator_.setModelType(pcl::SACMODEL_PLANE);
        // SAmple Consensus (SAC_) methods like RANSAC, LMEDS (Least Median of Squares)
        sac_estimator_.setMethodType(pcl::SAC_RANSAC);
        // Distance close to the model to be considered as an inlier, default 0
        sac_estimator_.setDistanceThreshold(params_.sac_distance_threshold);
        // Default 50
        //sac_estimator_.setMaxIterations(params_.sac_max_iteration);
        // Default 0.99
        //sac_estimator_.setProbability(params_.sac_probability);
    }

    GroundRANSACSegmenter::~GroundRANSACSegmenter()
    {}

    /**
     * @breif Ground Removal based-on Voxel Grid Filter
     * @note Cascade RANSAC is invalid...
     * @note Multi-segment RANSAC
     */
    const int segment_number = 3;
    int segments[segment_number] = {20, 30, 40};
    void GroundRANSACSegmenter::segment(const PointICloud& cloud_in,
                                        std::vector<PointICloudPtr>* cloud_clusters)
    {
        //Clear segments.
        (*cloud_clusters).clear();
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

            //extract ground points
            indiceExtractor.setNegative(false);
            indiceExtractor.filter(*cloud_ground);

            //extract non-ground points
            indiceExtractor.setNegative(true);
            indiceExtractor.filter(*cloud_nonground);

            ground_removed = true;
        }

        (*cloud_clusters).push_back(cloud_ground);
        (*cloud_clusters).push_back(cloud_nonground);

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
            //extract ground points
            indiceExtractor.setNegative(false);
            indiceExtractor.filter(*cloud_ground);
            //extract non-ground points
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

    //+3 makes sure the last segment contains last point
    int segment_size = points.size() / ground_removal_segment_size_ + 3;
    int segment_id = 1;
    cloud->clear();
    for (size_t pt = 0u; pt < points.size(); ++pt) {
        cloud->push_back(points[pt]);
        if (pt == segment_size*segment_id || pt == (points.size()-1)) {
            seg.setInputCloud(cloud);
            seg.segment(*ground_indices, *coefficients);

            if (ground_indices->indices.size() > 0) {
                pcl::ExtractIndices<Point> indiceExtractor;
                indiceExtractor.setInputCloud(cloud);
                indiceExtractor.setIndices(ground_indices);
                //extract ground points
                indiceExtractor.setNegative(false);
                indiceExtractor.filter(*cloud_ground);
                //extract non-ground points
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
        //extract ground points
        indiceExtractor.setNegative(false);
        indiceExtractor.filter(*cloud_ground);
        *pc_ground = *cloud_ground;

        //extract non-ground points
        // false: extract the inliers
        // true:	filter the inliers
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*cloud_nonground);

        //Try second ...
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

        ROS_INFO_STREAM("Ground removed. Took " << clock.takeRealTime() << " ms.");

        return true;
    } else {
        return false;
    }
#endif

        ROS_INFO_STREAM("Segmentation complete. Took " << clock.takeRealTime() << "ms.");
    }
}

//TODO Voxel Clustering & Progressive Morphological Filter
/*#include <pcl/segmentation/supervoxel_clustering.h> *//* pcl::SupervoxelClustering *//*
#include <pcl/segmentation/progressive_morphological_filter.h>
*//* pcl::ProgressiveMorphologicalFilter */
/*
    // Supervoxel Clustering
    float voxelseg_voxel_resolution_;
    float voxelseg_seed_resolution_;
    float voxelseg_spatial_importance_;
    float voxelseg_normal_importance_;
    // A Progressive Morphological Filter for Extracting Ground
    int pmfseg_max_window_size_;
    float pmfseg_initial_height_threshold_;
    float pmfseg_max_height_threshold_;
    float pmfseg_ground_slope_;

    float ground_removal_2nd_min_size_;
    int ground_removal_segment_size_;

bool LiDARTLD::groundSegmentationByPMF(const PointCloudPtr pc, PointCloudPtr pc_ground,
                                       PointCloudPtr pc_nonground)
{
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);

    common::Clock clock;
    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<Point> pmfSegmenter;
    pmfSegmenter.setMaxWindowSize(pmfseg_max_window_size_);
    pmfSegmenter.setInitialDistance(pmfseg_initial_height_threshold_);
    pmfSegmenter.setMaxDistance(pmfseg_max_height_threshold_);
    pmfSegmenter.setSlope(pmfseg_ground_slope_);
    ROS_INFO_STREAM("Init ProgressiveMorphologicalFilter took " << clock.takeRealTime() << " ms.");

    clock.start();
    pmfSegmenter.setInputCloud(pc);
    pmfSegmenter.extract(ground_indices->indices);
    ROS_INFO_STREAM("ProgressiveMorphologicalFilter extraction took " << clock.takeRealTime() << " ms.");

    if (ground_indices->indices.size() > 0) {
        //extract ground points
        pcl::copyPointCloud(*pc, *ground_indices, *pc_ground);

        //extract non-ground points
        pcl::ExtractIndices<Point> indiceExtractor;
        indiceExtractor.setInputCloud(pc);
        indiceExtractor.setIndices(ground_indices);
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*pc_nonground);

        return true;
    } else {
        return false;
    }
}

bool LiDARTLD::groundSegmentationByVoxel(const PointCloudPtr pc, PointCloudPtr pc_ground,
                                         PointCloudPtr pc_nonground)
{
    std::vector<PointCloudPtr> pc_clusters;

    pcl::SupervoxelClustering<Point>
            voxelSegmenter(voxelseg_voxel_resolution_, voxelseg_seed_resolution_);
    voxelSegmenter.setColorImportance(0.0f);
    voxelSegmenter.setSpatialImportance(voxelseg_spatial_importance_);
    voxelSegmenter.setNormalImportance(voxelseg_normal_importance_);

    PointCloudPtr cloud(new PointCloud);
    voxelGridFilter(pc, cloud);
    ROS_INFO_STREAM("After Voxel Grid filter: " << cloud->size());

    voxelSegmenter.setInputCloud(cloud);
    std::map<uint32_t, pcl::Supervoxel<Point>::Ptr> voxel_clusters;
    voxelSegmenter.extract(voxel_clusters);
    ROS_INFO_STREAM("After Voxel Clustering: " << voxel_clusters.size());

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    voxelSegmenter.getSupervoxelAdjacency(supervoxel_adjacency);
    std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
    for ( ; label_itr != supervoxel_adjacency.end(); ) {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<Point>::Ptr supervoxel = voxel_clusters.at(supervoxel_label);
        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudPtr adjacent_supervoxel_centers(new PointCloud);
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
            pcl::Supervoxel<Point>::Ptr neighbor_supervoxel = voxel_clusters.at (adjacent_itr->second);
            *adjacent_supervoxel_centers += *neighbor_supervoxel->voxels_;
        }
        pc_clusters.push_back(adjacent_supervoxel_centers);

        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    */
/*std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr>::iterator it;
    for(it = voxel_clusters.begin(); it != voxel_clusters.end(); ++it) {
        pc_clusters.push_back(it->second->voxels_);
    }*//*

    publisherClusters(pc_clusters);
    return true;
}*/
