#ifndef STEREO_CAMERA_NODE_H
#define STEREO_CAMERA_NODE_H

#include <ros/ros.h>
#include <opencv2/calib3d.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "undistortion.hpp"
#include "calibration.hpp"
#include "disparity_filters.hpp"

namespace rawseeds_ros {
class StereoMatcherNode
{
public:
    StereoMatcherNode();

    void run();

private:
    ros::NodeHandle             nh_;

    ros::Subscriber             sub_left_img_;
    ros::Subscriber             sub_right_img_;
    ros::Subscriber             sub_left_info_;
    ros::Subscriber             sub_right_info_;
    ros::Publisher              pub_points_;

    ros::Duration               pub_period_;
    ros::Time                   pub_last_time_;

    sensor_msgs::ImageConstPtr  left_img_;
    sensor_msgs::ImageConstPtr  right_img_;

    cv::Mat                     Q_;
    cv::Mat                     R_;
    cv::Mat                     T_;
    cv::Size                    S_;
    cv::Ptr<cv::StereoMatcher>  matcher_;
    double                      time_delta_;

    Calibration::Ptr            calibration_left_;
    Calibration::Ptr            calibration_right_;

    Undistortion::Ptr           undistortion_left_;
    Undistortion::Ptr           undistortion_right_;

    bool                                      debug_;
    bool                                      use_wls_filter_;
    double                                    wls_sigma_;
    double                                    wls_lambda_;

    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
    cv::Ptr<cv::StereoMatcher>                wls_right_matcher_;

    double                                    max_depth_;

    bool setup();

    void left(const sensor_msgs::ImageConstPtr &msg);
    void left(const sensor_msgs::CameraInfoConstPtr &msg);

    void right(const sensor_msgs::ImageConstPtr &msg);
    void right(const sensor_msgs::CameraInfoConstPtr &msg);

    void stereoRectify();
    void match();


};
}

#endif // STEREO_CAMERA_NODE_H
