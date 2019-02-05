/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_SEGMENTER_MANAGER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_SEGMENTER_MANAGER_HPP_

#include <ros/ros.h>
#include <memory>

#include "common/types/type.h"

#include "segmenters/base_segmenter.hpp"
// ground segmenters
#include "segmenters/ground_plane_fitting_segmenter.hpp"
#include "segmenters/ground_ransac_segmenter.hpp"
// non-ground segmenters
#include "segmenters/don_segmenter.hpp"
#include "segmenters/euclidean_segmenter.hpp"
#include "segmenters/region_euclidean_segmenter.hpp"
#include "segmenters/region_growing_segmenter.hpp"

namespace autosense {
namespace segmenter {
/*
 * @brief create ground remover
 */
static std::unique_ptr<BaseSegmenter> createGroundSegmenter(
    const SegmenterParams &params) {
    std::unique_ptr<BaseSegmenter> segmenter;
    if (params.segmenter_type == "GroundPlaneFittingSegmenter") {
        segmenter = std::unique_ptr<BaseSegmenter>(
            new GroundPlaneFittingSegmenter(params));
        ROS_INFO("[segment] Instance of GPF Ground Segmenter created.");
    } else if (params.segmenter_type == "GroundRANSACSegmenter") {
        segmenter =
            std::unique_ptr<BaseSegmenter>(new GroundRANSACSegmenter(params));
        ROS_INFO("[segment] Instance of RANSAC Ground Segmenter created.");
    } else {
        ROS_ERROR_STREAM("The ground remover " << params.segmenter_type
                                               << " was not implemented.");
    }
    return segmenter;
}

/*
 * @brief create Segmenter
 *  Euclidean Segmenter
 *  Region Growing Segmenter
 */
static std::unique_ptr<BaseSegmenter> createNonGroundSegmenter(
    const SegmenterParams &params) {
    std::unique_ptr<BaseSegmenter> segmenter;
    if (params.segmenter_type == "RegionEuclideanSegmenter") {
        segmenter = std::unique_ptr<BaseSegmenter>(
            new RegionEuclideanSegmenter(params));
        ROS_INFO(
            "[segment] Instance of Region Euclidean Non-ground Segmenter "
            "created.");
    } else if (params.segmenter_type == "EuclideanSegmenter") {
        segmenter =
            std::unique_ptr<BaseSegmenter>(new EuclideanSegmenter(params));
        ROS_INFO(
            "[segment] Instance of Euclidean Non-ground Segmenter created.");
    } else if (params.segmenter_type == "RegionGrowingSegmenter") {
        segmenter =
            std::unique_ptr<BaseSegmenter>(new RegionGrowingSegmenter(params));
    } else if (params.segmenter_type == "DoNSegmenter") {
        segmenter = std::unique_ptr<BaseSegmenter>(new DoNSegmenter(params));
    } else {
        ROS_ERROR_STREAM("The segmenter " << params.segmenter_type
                                          << " was not implemented.");
    }

    return segmenter;
}

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_SEGMENTER_MANAGER_HPP_
