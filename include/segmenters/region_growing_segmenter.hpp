#ifndef _REGION_GROWING_SEGMENTER_HPP_
#define _REGION_GROWING_SEGMENTER_HPP_

#include "./base_segmenter.hpp"

#include <pcl/search/kdtree.h>                          /* pcl::search::KdTree */
#include <pcl/features/normal_3d_omp.h>                 /* pcl::NormalEstimationOMP */
#include <pcl/segmentation/region_growing.h>            /* pcl::RegionGrowing */

namespace segmenter {
    class RegionGrowingSegmenter : public BaseSegmenter {
    public:
        RegionGrowingSegmenter();

        explicit RegionGrowingSegmenter(const SegmenterParams& params);

        ~RegionGrowingSegmenter();

        /// \brief Segment the point cloud.
        virtual void segment(const PointCloudPtr pc, std::vector<PointCloudPtr>& pc_clusters);

        virtual std::string name() const
        {
            return "RegionGrowingSegmenter";
        }

    private:
        SegmenterParams params_;

        /**
         * @breif For our later calls to calculate normals, we need to create a search tree.
         * @note
         *  Organized data (i.e. a depth image): OrganizedNeighbor search tree
         *  Unorganized data (i.e. LiDAR scans): KDTree
         */
        pcl::search::KdTree<Point>::Ptr kd_tree_;
        /**
         * @brief makes use of OpenMP to use many threads to calculate the normal
         * @note estimating the normals, also the bottleneck computationally
         *  standard single-threaded class: pcl::NormalEstimation
         *  GPU accelerated class: pcl::gpu::NormalEstimation
         */
        pcl::NormalEstimationOMP<Point, Normal> normal_estimator_omp_;
        /**
         *
         */
        pcl::RegionGrowing<Point, Normal> region_growing_estimator_;
    }; /* class RegionGrowingSegmenter */
}

#endif /* _REGION_GROWING_SEGMENTER_HPP_ */
