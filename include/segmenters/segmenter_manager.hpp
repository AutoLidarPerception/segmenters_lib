#ifndef _SEGMENTER_MANAGER_HPP_
#define _SEGMENTER_MANAGER_HPP_

#include "common/types/type.h"
#include <ros/ros.h>

#include "./base_segmenter.hpp"
//ground segmenters
#include "./ground_plane_fitting_segmenter.hpp"
#include "./ground_ransac_segmenter.hpp"
//non-ground segmenters
#include "./region_euclidean_segmenter.hpp"
#include "./euclidean_segmenter.hpp"
#include "./don_segmenter.hpp"
#include "./region_growing_segmenter.hpp"

namespace segmenter {
    /*
     * @brief create Segmenter
     *  Euclidean Segmenter
     *  Region Growing Segmenter
     */
    static std::unique_ptr<BaseSegmenter> createNonGroundSegmenter(SegmenterParams params)
    {
        std::unique_ptr<BaseSegmenter> segmenter;
        if (params.segmenter_type == "RegionEuclideanSegmenter") {
            segmenter = std::unique_ptr<BaseSegmenter>(new RegionEuclideanSegmenter(params));
            ROS_INFO("[segment] Instance of Region Euclidean Non-ground Segmenter created.");
        } else if (params.segmenter_type == "EuclideanSegmenter") {
            segmenter = std::unique_ptr<BaseSegmenter>(new EuclideanSegmenter(params));
            ROS_INFO("[segment] Instance of Euclidean Non-ground Segmenter created.");
        /*} else if (params.segmenter_type == "RegionGrowingSegmenter") {
            segmenter = std::unique_ptr<Segmenter>(new RegionGrowingSegmenter(params));
        } else if (params.segmenter_type == "DoNSegmenter") {
            segmenter = std::unique_ptr<Segmenter>(new DoNSegmenter(params));*/
        } else {
            ROS_ERROR_STREAM("The segmenter " << params.segmenter_type << " was not implemented.");
        }

        return segmenter;
    }

    /*
     * @brief create ground remover
     */
    static std::unique_ptr<BaseSegmenter> createGroundSegmenter(SegmenterParams params)
    {
        std::unique_ptr<BaseSegmenter> segmenter;
        if (params.segmenter_type == "GroundPlaneFittingSegmenter") {
            segmenter = std::unique_ptr<BaseSegmenter>(new GroundPlaneFittingSegmenter(params));
            ROS_INFO("[segment] Instance of GPF Ground Segmenter created.");
        } else if (params.segmenter_type == "GroundRANSACSegmenter") {
            segmenter = std::unique_ptr<BaseSegmenter>(new GroundRANSACSegmenter(params));
            ROS_INFO("[segment] Instance of RANSAC Ground Segmenter created.");
        } else {
            ROS_ERROR_STREAM("The ground remover " << params.segmenter_type << " was not implemented.");
        }
        return segmenter;
    }
}

#endif /* _SEGMENTER_MANAGER_HPP_ */
