/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/ground_plane_fitting_segmenter.hpp"

#include <ros/ros.h>

#include <pcl/filters/extract_indices.h>  // pcl::ExtractIndices
#include <pcl/io/io.h>                    // pcl::copyPointCloud

#include "common/common.hpp"  // common::sortByAxisZAsc
#include "common/time.hpp"    // common::Clock

namespace autosense {
namespace segmenter {

GroundPlaneFittingSegmenter::GroundPlaneFittingSegmenter() {}

GroundPlaneFittingSegmenter::GroundPlaneFittingSegmenter(
    const SegmenterParams& params)
    : params_(params) {}

GroundPlaneFittingSegmenter::~GroundPlaneFittingSegmenter() {}

/**
 * @brief Selection of initial seed points
 * @note
 *  Introduces the lowest point representative(LPR)
 *      ==> guarantees that noisy measurements will not affect the plane
 * estimation step
 * @input
 *  params_.gpf_num_lpr: number of  lowest point representative(LPR),
 * @param cloud_in
 * @param cloud_seeds
 */
void GroundPlaneFittingSegmenter::extractInitialSeeds(
    const PointICloud& cloud_in, PointICloudPtr cloud_seeds) {
    std::vector<PointI> points(cloud_in.points.begin(), cloud_in.points.end());
    std::sort(points.begin(), points.end(), common::sortByAxisZAsc<PointI>);

    int cnt_lpr = 0;
    double height_average = 0.;
    // filter negative obstacles
    bool negative = true;
    for (size_t pt = 0u; pt < points.size() && cnt_lpr < params_.gpf_num_lpr;
         ++pt) {
        const double& h = points[pt].z;
        if (negative) {
            if (fabs(h + params_.gpf_sensor_height) > params_.gpf_th_lprs) {
                continue;
            } else {
                // because points are in "Incremental Order"
                negative = false;
            }
        }
        // collect from non-negative obstacles
        height_average += h;
        cnt_lpr++;
    }

    if (cnt_lpr > 0) {
        height_average /= cnt_lpr;
    } else {
        height_average = (-1.0) * params_.gpf_sensor_height;
    }

    // the points inside the height threshold are used as the initial seeds for
    // the plane model estimation
    (*cloud_seeds).clear();
    for (size_t pt = 0u; pt < points.size(); ++pt) {
        if (points[pt].z < height_average + params_.gpf_th_seeds) {
            (*cloud_seeds).points.push_back(points[pt]);
        }
    }
}

/**
 * @brief Estimate the ground plane(N^T X = -d) by SVD
 * @param cloud_seeds
 * @retval plane model estimation model_t
 */
model_t GroundPlaneFittingSegmenter::estimatePlane(
    const PointICloud& cloud_ground) {
    model_t model;

    // Create covariance matrix.
    // 1. calculate (x,y,z) mean
    float mean_x = 0., mean_y = 0., mean_z = 0.;
    for (size_t pt = 0u; pt < cloud_ground.points.size(); ++pt) {
        mean_x += cloud_ground.points[pt].x;
        mean_y += cloud_ground.points[pt].y;
        mean_z += cloud_ground.points[pt].z;
    }
    if (cloud_ground.points.size()) {
        mean_x /= cloud_ground.points.size();
        mean_y /= cloud_ground.points.size();
        mean_z /= cloud_ground.points.size();
    }
    // 2. calculate covariance
    // cov(x,x), cov(y,y), cov(z,z)
    // cov(x,y), cov(x,z), cov(y,z)
    float cov_xx = 0., cov_yy = 0., cov_zz = 0.;
    float cov_xy = 0., cov_xz = 0., cov_yz = 0.;
    for (int i = 0; i < cloud_ground.points.size(); i++) {
        cov_xx += (cloud_ground.points[i].x - mean_x) *
                  (cloud_ground.points[i].x - mean_x);
        cov_xy += (cloud_ground.points[i].x - mean_x) *
                  (cloud_ground.points[i].y - mean_y);
        cov_xz += (cloud_ground.points[i].x - mean_x) *
                  (cloud_ground.points[i].z - mean_z);
        cov_yy += (cloud_ground.points[i].y - mean_y) *
                  (cloud_ground.points[i].y - mean_y);
        cov_yz += (cloud_ground.points[i].y - mean_y) *
                  (cloud_ground.points[i].z - mean_z);
        cov_zz += (cloud_ground.points[i].z - mean_z) *
                  (cloud_ground.points[i].z - mean_z);
    }
    // 3. setup covariance matrix Cov
    Eigen::MatrixXf Cov(3, 3);
    Cov << cov_xx, cov_xy, cov_xz, cov_xy, cov_yy, cov_yz, cov_xz, cov_yz,
        cov_zz;
    Cov /= cloud_ground.points.size();

    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> SVD(
        Cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    model.normal = (SVD.matrixU().col(2));
    // d is directly computed substituting x with s^ which is a good
    // representative for the points belonging to the plane
    Eigen::MatrixXf mean_seeds(3, 1);
    mean_seeds << mean_x, mean_y, mean_z;
    // according to normal^T*[x,y,z]^T = -d
    model.d = -(model.normal.transpose() * mean_seeds)(0, 0);

    // ROS_WARN_STREAM("Model: " << model.normal << " " << model.d);

    return model;
}

/**
 * @brief GPF main loop for one segemnt of the point cloud
 * @param cloud_in
 * @param cloud_gnds
 * @param cloud_ngnds
 */
void GroundPlaneFittingSegmenter::mainLoop(const PointICloud& cloud_in,
                                           PointICloudPtr cloud_gnds,
                                           PointICloudPtr cloud_ngnds) {
    cloud_gnds->clear();
    cloud_ngnds->clear();

    PointICloudPtr cloud_seeds(new PointICloud);
    extractInitialSeeds(cloud_in, cloud_seeds);

    pcl::PointIndices::Ptr gnds_indices(new pcl::PointIndices);
    *cloud_gnds = *cloud_seeds;
    for (size_t iter = 0u; iter < params_.gpf_num_iter; ++iter) {
        model_t model = estimatePlane(*cloud_gnds);
        // clear
        cloud_gnds->clear();
        gnds_indices->indices.clear();
        // pointcloud to matrix
        Eigen::MatrixXf cloud_matrix(cloud_in.points.size(), 3);
        size_t pi = 0u;
        for (auto p : cloud_in.points) {
            cloud_matrix.row(pi++) << p.x, p.y, p.z;
        }
        // distance to extimated ground plane model (N^T X)^T = (X^T N)
        Eigen::VectorXf dists = cloud_matrix * model.normal;
        // threshold filter: N^T xi + d = dist < th_dist ==> N^T xi < th_dist -
        // d
        double th_dist = params_.gpf_th_gnds - model.d;
        for (size_t pt = 0u; pt < dists.rows(); ++pt) {
            if (dists[pt] < th_dist) {
                gnds_indices->indices.push_back(pt);
            }
        }
        // extract ground points
        pcl::copyPointCloud(cloud_in, *gnds_indices, *cloud_gnds);
    }

    // extract non-ground points
    pcl::ExtractIndices<PointI> indiceExtractor;
    indiceExtractor.setInputCloud(cloud_in.makeShared());
    indiceExtractor.setIndices(gnds_indices);
    indiceExtractor.setNegative(true);
    indiceExtractor.filter(*cloud_ngnds);
}

void GroundPlaneFittingSegmenter::segment(
    const PointICloud& cloud_in, std::vector<PointICloudPtr>& cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty ground for segmentation, do nonthing.");
        return;
    }
    // Clear segments.
    cloud_clusters.clear();

    common::Clock clock;
    ROS_INFO("Starting Ground Plane Fitting segmentation.");

    // Main Loop
    PointICloudPtr cloud_ground(new PointICloud),
        cloud_nonground(new PointICloud);
    if (params_.gpf_num_segment > 1) {
        // divide into multi-segments by X-axis
        boost::array<std::vector<int>, segment_upper_bound_> segment_indices;

        std::vector<PointI> points(cloud_in.points.begin(),
                                   cloud_in.points.end());
        std::sort(points.begin(), points.end(), common::sortByAxisXAsc<PointI>);
        const double res =
            ((--points.end())->x - points.begin()->x) / params_.gpf_num_segment;
        for (size_t pt = 0u; pt < cloud_in.points.size(); ++pt) {
            double Xval = points.begin()->x;
            for (size_t idx = 0u; idx < params_.gpf_num_segment; ++idx) {
                const double x = cloud_in.points[pt].x;
                if (x > Xval && x < (Xval + res)) {
                    segment_indices[idx].push_back(pt);
                }
                Xval += res;
            }
        }

        PointICloudPtr cloud(new PointICloud);
        PointICloudPtr gnds(new PointICloud), ngnds(new PointICloud);
        for (size_t segmentIdx = 0u; segmentIdx < params_.gpf_num_segment;
             ++segmentIdx) {
            // get pointcloud in different segments
            pcl::copyPointCloud(cloud_in, segment_indices[segmentIdx], *cloud);

            mainLoop(*cloud, gnds, ngnds);

            *cloud_ground += *gnds;
            *cloud_nonground += *ngnds;
        }
    } else {
        mainLoop(cloud_in, cloud_ground, cloud_nonground);
    }

    // construct ground/non-ground clusters
    cloud_clusters.push_back(cloud_ground);
    cloud_clusters.push_back(cloud_nonground);

    ROS_INFO_STREAM("GPF Segmentation complete. Took " << clock.takeRealTime()
                                                       << "ms.");
}

}  // namespace segmenter
}  // namespace autosense
