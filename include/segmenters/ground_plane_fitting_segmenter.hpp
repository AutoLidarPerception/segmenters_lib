/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_PLANE_FITTING_SEGMENTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_PLANE_FITTING_SEGMENTER_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "segmenters/base_segmenter.hpp"

namespace autosense {
namespace segmenter {

typedef struct {
    Eigen::MatrixXf normal;
    double d = 0.;
} model_t;

/**
 * @brief Ground Removal based on Ground Plane Fitting(GPF)
 * @refer
 *   Fast Segmentation of 3D Point Clouds: A Paradigm on LiDAR Data for
 * Autonomous Vehicle Applications (ICRA, 2017)
 */
class GroundPlaneFittingSegmenter : public BaseSegmenter {
 public:
    GroundPlaneFittingSegmenter();

    explicit GroundPlaneFittingSegmenter(const SegmenterParams &params);

    ~GroundPlaneFittingSegmenter();

    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud &cloud_in,
        std::vector<PointICloudPtr> &cloud_clusters);  // NOLINT

    virtual std::string name() const { return "GroundPlaneFittingSegmenter"; }

 private:
    void extractInitialSeeds(const PointICloud &cloud_in,
                             PointICloudPtr cloud_seeds);

    model_t estimatePlane(const PointICloud &cloud_ground);

    void mainLoop(const PointICloud &cloud_in,
                  PointICloudPtr cloud_gnds,
                  PointICloudPtr cloud_ngnds);

 private:
    SegmenterParams params_;
    static const int segment_upper_bound_ = 6;
};  // class GroundPlaneFittingSegmenter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_PLANE_FITTING_SEGMENTER_HPP_
