#ifndef _EUCLIDEAN_SEGMENTER_HPP_
#define _EUCLIDEAN_SEGMENTER_HPP_

#include "./base_segmenter.hpp"

#include <pcl/search/kdtree.h>                  /* pcl::search::KdTree */
#include <pcl/segmentation/extract_clusters.h>  /* pcl::EuclideanClusterExtraction */

namespace segmenter {

    class EuclideanSegmenter : public BaseSegmenter {
    public:
        EuclideanSegmenter();

        explicit EuclideanSegmenter(const SegmenterParams& params);

        ~EuclideanSegmenter();

        /// \brief Segment the point cloud.
        virtual void segment(const PointICloud& cloud_in,
                             std::vector<PointICloudPtr>* cloud_clusters);

        virtual std::string name() const
        {
            return "EuclideanSegmenter";
        }

    private:
        SegmenterParams params_;

        pcl::search::KdTree<PointI>::Ptr kd_tree_;
        pcl::EuclideanClusterExtraction<PointI> euclidean_cluster_extractor_;
    }; /* class EuclideanSegmenter */
}

#endif /* _EUCLIDEAN_SEGMENTER_HPP_ */
