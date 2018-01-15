#ifndef RAWSEEDS_ROS_UNDISTORTION_HPP
#define RAWSEEDS_ROS_UNDISTORTION_HPP

#include <opencv2/opencv.hpp>

namespace rawseeds_ros {
class Undistortion
{
public:
    Undistortion(const cv::Size size,
                 const cv::Mat &intrinsics,
                 const cv::Mat &distortion,
                 const cv::Mat &R,
                 const cv::Mat &P)
    {
        cv::initUndistortRectifyMap(intrinsics, distortion, R, P, size, CV_16SC2, map_x_, map_y_);
    }

    void apply(cv::Mat &src, cv::Mat &dst)
    {
        cv::remap(src, dst, map_1_, map_2_, CV_INTER_LINEAR);
    }

private:
    cv::Mat map_1_;
    cv::Mat map_2_;

};
}


#endif // RAWSEEDS_ROS_UNDISTORTION_HPP
