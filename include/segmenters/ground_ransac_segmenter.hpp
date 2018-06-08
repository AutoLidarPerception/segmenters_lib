#ifndef _GROUNG_RANSAC_SEGMENTER_HPP_
#define _GROUNG_RANSAC_SEGMENTER_HPP_

#include "./base_segmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>        /* pcl::SACSegmentation */
#include "common/types/type.h"

namespace segmenter {

    class GroundRANSACSegmenter : public BaseSegmenter {
    public:
        GroundRANSACSegmenter();

        explicit GroundRANSACSegmenter(const SegmenterParams& params);

        ~GroundRANSACSegmenter();

        /// \brief Segment the point cloud.
        virtual void segment(const PointICloud& cloud_in,
                             std::vector<PointICloudPtr>* cloud_clusters);

        virtual std::string name() const
        {
            return "GroundRANSACSegmenter";
        }

    private:
        SegmenterParams params_;

        pcl::SACSegmentation<PointI> sac_estimator_;
    }; /* class GroundRANSACSegmenter */
}

#endif /* _GROUNG_RANSAC_SEGMENTER_HPP_ */
