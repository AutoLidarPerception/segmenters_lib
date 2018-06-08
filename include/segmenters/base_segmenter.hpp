#ifndef _BASE_SEGMENTER_HPP_
#define _BASE_SEGMENTER_HPP_

#include "common/types/type.h"

#include <ros/ros.h>
#include <string>

namespace segmenter {

    class BaseSegmenter {
    public:
        /// \brief Segment the point cloud.
        virtual void segment(const PointICloud& cloud_in,
                             std::vector<PointICloudPtr>* cloud_clusters) = 0;

        virtual std::string name() const = 0;
    }; /* BaseSegmenter */
}

#endif /* _BASE_SEGMENTER_HPP_ */
