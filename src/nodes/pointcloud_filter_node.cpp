#include "pointcloud_filter_node.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "voxel.hpp"

namespace rawseeds_ros {
PointcloudFilterNode::PointcloudFilterNode() :
    nh_("~")
{

}
PointcloudFilterNode::~PointcloudFilterNode()
{
}


void PointcloudFilterNode::run()
{
    setup();
    ros::spin();
}


void PointcloudFilterNode::setup()
{
    filter_frame_ = nh_.param<std::string>("filter_frame", "base_link");

    volume_min_[0] = nh_.param<double>("volume/min/x", -10.0);
    volume_min_[1] = nh_.param<double>("volume/min/y", -10.0);
    volume_min_[2] = nh_.param<double>("volume/min/z",   0.0);
    volume_max_[0] = nh_.param<double>("volume/min/x",  10.0);
    volume_max_[1] = nh_.param<double>("volume/min/y",  10.0);
    volume_max_[2] = nh_.param<double>("volume/min/z",   4.0);

    voxel_resolution_[0] = nh_.param<double>("voxel/resolution/x", 0.1);
    voxel_resolution_[1] = nh_.param<double>("voxel/resolution/y", 0.1);
    voxel_resolution_[2] = nh_.param<double>("voxel/resolution/z", 0.1);

    const std::string topic_pointcloud      = nh_.param<std::string>("topic_pointcloud", "/svs_l/points");
    const std::string topic_filtered_volume = nh_.param<std::string>("topic_volume_filtered", "svs_l/filtered");
    const std::string topic_filtered_voxel  = nh_.param<std::string>("topic_volume_filtered", "svs_l/filtered_voxels");

    sub_pointcloud_ = nh_.subscribe(topic_pointcloud, 1, &PointcloudFilterNode::pointcloud, this);
    pub_volume_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_filtered_volume, 1);
    pub_voxel_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_filtered_voxel, 1);

}

void PointcloudFilterNode::pointcloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PCLPointCloud2 pcl_pts2;
    pcl_conversions::toPCL(*msg, pcl_pts2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pts(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pts2,*pcl_pts);

    /// filter
    volumeFilter(pcl_pts, pcl_pts);
    if(pub_volume_filtered_.getNumSubscribers() > 0) {
        publish(pcl_pts, pub_volume_filtered_);
    }
    if(pub_voxel_filtered_.getNumSubscribers() > 0) {
        voxelFilter(pcl_pts, pcl_pts);
        publish(pcl_pts, pub_voxel_filtered_);
    }
}

void PointcloudFilterNode::publish(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out,
                                   ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*out, msg_out);
    pub.publish(msg_out);
}

void PointcloudFilterNode::volumeFilter(const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    filtered.reset(new  pcl::PointCloud<pcl::PointXYZRGB>);
    filtered->header   = src->header;
    filtered->is_dense = true;

    auto in_x_limits = [this](const tf::Point &p)
    {
        return p.x() >= volume_min_[0] && p.x() <= volume_max_[0];
    };
    auto in_y_limites = [this](const tf::Point &p)
    {
        return p.y() >= volume_min_[1] && p.y() <= volume_max_[1];
    };
    auto in_z_limites = [this](const tf::Point &p)
    {
        return p.z() >= volume_min_[2] && p.z() <= volume_max_[2];
    };

    ros::Time stamp ;
    pcl_conversions::fromPCL(src->header.stamp, stamp);
    if(filter_frame_ != src->header.frame_id) {
        tf::StampedTransform t;
        if(tf_.waitForTransform(filter_frame_, src->header.frame_id, stamp, ros::Duration(0.1))) {
            tf_.lookupTransform(filter_frame_, src->header.frame_id, stamp, t);
            for(const pcl::PointXYZRGB &p_src : src->points) {
                const tf::Point p_tf = t * tf::Point(p_src.x, p_src.y, p_src.z);
                if(in_x_limits(p_tf) && in_y_limites(p_tf) && in_z_limites(p_tf)) {
                    filtered->points.emplace_back(p_src);
                }
            }
        }
    } else {
        for(const pcl::PointXYZRGB &p_src : src->points) {
            const tf::Point p_tf = tf::Point(p_src.x, p_src.y, p_src.z);
            if(in_x_limits(p_tf) && in_y_limites(p_tf) && in_z_limites(p_tf)) {
                filtered->points.emplace_back(p_src);
            }
        }
    }
    std::swap(dst, filtered);
}

void PointcloudFilterNode::voxelFilter(const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst)
{
    /// voxel filter the shit
    std::shared_ptr<VoxelGrid> voxel_grid(new VoxelGrid);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    filtered.reset(new  pcl::PointCloud<pcl::PointXYZRGB>);
    filtered->header   = src->header;
    filtered->is_dense = true;

    auto index = [this](const tf::Point &pt) {
        return Voxel::index_t {{static_cast<int>(pt.x() / voxel_resolution_[0]),
                                static_cast<int>(pt.y() / voxel_resolution_[1]),
                                static_cast<int>(pt.z() / voxel_resolution_[2])}};
    };

    ros::Time stamp ;
    pcl_conversions::fromPCL(src->header.stamp, stamp);
    if(filter_frame_ != src->header.frame_id) {
        tf::StampedTransform t;
        if(tf_.waitForTransform(filter_frame_, src->header.frame_id, stamp, ros::Duration(0.1))) {
            tf_.lookupTransform(filter_frame_, src->header.frame_id, stamp, t);
            for(const pcl::PointXYZRGB &p_src : src->points) {
                const tf::Point p_pcl  = tf::Point(p_src.x, p_src.y, p_src.z);
                const tf::Point p_tf = t * p_pcl;
                const tf::Point c_tf = tf::Point(p_src.r, p_src.g, p_src.b);
                voxel_grid->insert(index(p_pcl), Voxel(p_tf, c_tf));
            }
        }
    } else {
        for(const pcl::PointXYZRGB &p_src : src->points) {
            const tf::Point p_tf = tf::Point(p_src.x, p_src.y, p_src.z);
            const tf::Point c_tf = tf::Point(p_src.r, p_src.g, p_src.b);
            voxel_grid->insert(index(p_tf), Voxel(p_tf, c_tf));
        }
    }

    auto add_point = [&filtered](const Voxel::index_t &i, const Voxel &v) {
        const tf::Point &p_tf = v.mean();
        const tf::Point &c_tf = v.color();
        pcl::PointXYZRGB p_pcl;
        p_pcl.x = p_tf.x();
        p_pcl.y = p_tf.y();
        p_pcl.z = p_tf.z();
        p_pcl.r = c_tf.x();
        p_pcl.g = c_tf.y();
        p_pcl.b = c_tf.z();
        filtered->points.emplace_back(p_pcl);
    };
    voxel_grid->traverse(add_point);
    std::swap(dst, filtered);
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rawseeds_ros_stereo_matcher_node");
    rawseeds_ros::PointcloudFilterNode f;
    f.run();
    return 0;
}


