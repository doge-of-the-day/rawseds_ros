#ifndef POINTCLOUD_FILTER_NODE_H
#define POINTCLOUD_FILTER_NODE_H

#include <array>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

namespace rawseeds_ros {
class PointcloudFilterNode
{
public:
    PointcloudFilterNode();
    virtual ~PointcloudFilterNode();

    void run();

private:
    std::string           filter_frame_;

    std::array<double, 3> volume_min_;
    std::array<double, 3> volume_max_;
    std::array<double, 3> voxel_resolution_;

    ros::NodeHandle         nh_;
    ros::Subscriber         sub_pointcloud_;
    ros::Publisher          pub_volume_filtered_;
    ros::Publisher          pub_voxel_filtered_;
    tf::TransformListener   tf_;

    void setup();

    void pointcloud(const sensor_msgs::PointCloud2ConstPtr &msg);

    void publish(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out,
                 ros::Publisher &pub);

    void volumeFilter(const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst);

    void voxelFilter(const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst);

};
}

#endif // POINTCLOUD_FILTER_NODE_H
