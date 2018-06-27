#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>    /* pcl::fromROSMsg */

#include "common/types/type.h"                  /* PointICloudPtr */
#include "common/parameter.hpp"                 /* common::getSegmenterParams */
#include "segmenters/segmenter_manager.hpp"     /* segmenter::createGroundSegmenter */
#include "common/time.hpp"                      /* common::Clock */
#include "common/publisher.hpp"                 /* common::publishCloud */

#include "roi_filters/roi.hpp"                  /* roi::applyROIFilter */

const std::string param_ns_prefix_ = "/detect";
std::string frame_id_;
bool use_roi_filter_;
ROIParams params_roi_;
bool use_non_ground_segmenter_;
// ROS Subscriber
ros::Subscriber pointcloud_sub_;
// ROS Publisher
ros::Publisher ground_pub_;
ros::Publisher nonground_pub_;
ros::Publisher clusters_pub_;
///@note Core components
boost::shared_ptr<segmenter::BaseSegmenter> ground_remover_;
boost::shared_ptr<segmenter::BaseSegmenter> segmenter_;


void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr& ros_pc2)
{
    common::Clock clock;

    PointICloudPtr cloud(new PointICloud);
    pcl::fromROSMsg(*ros_pc2, *cloud);
    ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();

    if (use_roi_filter_) {
        roi::applyROIFilter<PointI>(params_roi_, cloud);
    }

    std::vector<PointICloudPtr> cloud_clusters;
    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);

    ground_remover_->segment(*cloud, &cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // reset clusters
    cloud_clusters.clear();

    if (use_non_ground_segmenter_) {
        segmenter_->segment(*cloud_nonground, &cloud_clusters);
        common::publishClustersCloud<PointI>(clusters_pub_, header, cloud_clusters);
    }


    common::publishCloud<PointI>(ground_pub_, header, *cloud_ground);
    common::publishCloud<PointI>(nonground_pub_, header, *cloud_nonground);

    ROS_INFO_STREAM("Cloud processed. Took " << clock.takeRealTime() << "ms.\n");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_node");
    
    // Node handle
    ros::NodeHandle nh = ros::NodeHandle(), private_nh = ros::NodeHandle("~");

    //@brief Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);

    std::string sub_pc_topic,
                pub_pc_ground_topic,
                pub_pc_nonground_topic, pub_pc_clusters_topic;
    int sub_pc_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size", sub_pc_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_pc_ground_topic", pub_pc_ground_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_pc_nonground_topic", pub_pc_nonground_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_pc_clusters_topic", pub_pc_clusters_topic);

    ///@note Important to use roi filter for "Ground remover"
    private_nh.param<bool>(param_ns_prefix_ + "/use_roi_filter", use_roi_filter_, false);
    params_roi_ = common::getRoiParams(private_nh, param_ns_prefix_);

    // Ground remover & non-ground segmenter
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type", ground_remover_type, "GroundPlaneFittingSegmenter");
    private_nh.param<bool>(param_ns_prefix_ + "/use_non_ground_segmenter", use_non_ground_segmenter_, false);
    private_nh.param<std::string>(param_ns_prefix_ + "/non_ground_segmenter_type", non_ground_segmenter_type, "RegionEuclideanSegmenter");
    SegmenterParams param = common::getSegmenterParams(private_nh, param_ns_prefix_);

    param.segmenter_type = ground_remover_type;
    ground_remover_ = segmenter::createGroundSegmenter(param);

    if (use_non_ground_segmenter_) {
        param.segmenter_type = non_ground_segmenter_type;
        segmenter_ = segmenter::createNonGroundSegmenter(param);
    }

    ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_pc_ground_topic, 1);
    nonground_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_pc_nonground_topic, 1);
    clusters_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_pc_clusters_topic, 1);

    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(sub_pc_topic, sub_pc_queue_size, OnPointCloud);

    ROS_INFO("detection_node started...");

    ros::Rate fps(40);
    while (ros::ok()) {
        ros::spinOnce();
        fps.sleep();
    }

    ROS_INFO("detection_node exited...");

    return 0;
}