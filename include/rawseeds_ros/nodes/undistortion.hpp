#ifndef RAWSEEDS_ROS_UNDISTORTION_HPP
#define RAWSEEDS_ROS_UNDISTORTION_HPP

#include <opencv2/opencv.hpp>
#include <memory>

namespace rawseeds_ros {

#if CV_VERSION_MAJOR >= 4
static const auto CV_UNDISTORTION_INTERPOLATION_METHOD = cv::INTER_LINEAR;
#else
static const auto CV_UNDISTORTION_INTERPOLATION_METHOD = CV_INTER_LINEAR;
#endif

class Undistortion
{
public:
    using Ptr = std::shared_ptr<Undistortion>;

    Undistortion(const cv::Size size,
                 const cv::Mat &intrinsics,
                 const cv::Mat &distortion,
                 const cv::Mat &R,
                 const cv::Mat &P) :
        mask_(size.height, size.width, CV_8UC1, cv::Scalar::all(255))
    {
        cv::initUndistortRectifyMap(intrinsics, distortion, R, P, size, CV_16SC2, map_1_, map_2_);
        apply(mask_, mask_);
    }

    void apply(cv::Mat &src, cv::Mat &dst)
    {
        cv::remap(src, dst, map_1_, map_2_, CV_UNDISTORTION_INTERPOLATION_METHOD);
    }

    const cv::Mat & getMask() const
    {
        return mask_;
    }

private:
    cv::Mat map_1_;
    cv::Mat map_2_;
    cv::Mat mask_;
};
}


#endif // RAWSEEDS_ROS_UNDISTORTION_HPP
