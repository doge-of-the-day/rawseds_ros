#ifndef STEREO_CAMERA_NODE_H
#define STEREO_CAMERA_NODE_H

#include <ros/ros.h>
#include <opencv2/calib3d.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace rawseeds_ros {
class StereoCameraNode
{
public:
    StereoCameraNode();

    void run();

private:
    ros::NodeHandle             nh_;

    ros::Subscriber             sub_left_img_;
    ros::Subscriber             sub_right_img_;
    ros::Subscriber             sub_left_info_;
    ros::Subscriber             sub_right_info_;

    ros::Time                   left_stamp_;
    ros::Time                   right_stamp_;
    cv::Mat                     left_img_;
    cv::Mat                     right_img_;


    cv::Ptr<cv::StereoMatcher>  matcher_;

    bool setup();

    void left(const sensor_msgs::ImageConstPtr &msg);
    void left(const sensor_msgs::CameraInfoConstPtr &msg);

    void right(const sensor_msgs::ImageConstPtr &msg);
    void right(const sensor_msgs::CameraInfoConstPtr &msg);


};
}

#endif // STEREO_CAMERA_NODE_H
