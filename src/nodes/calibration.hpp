#ifndef RAWSEEDS_ROS_CALIBRATION_HPP
#define RAWSEEDS_ROS_CALIBRATION_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

namespace rawseeds_ros {

class Calibration {
public:
    using Ptr = std::shared_ptr<Calibration>;

    Calibration(const sensor_msgs::CameraInfoConstPtr &info) :
        size_(info->width, info->height)
    {
        K_ = cv::Mat(3,3, CV_64FC1, cv::Scalar());
        D_ = cv::Mat(5,1, CV_64FC1, cv::Scalar());
        R_ = cv::Mat(3,3, CV_64FC1, cv::Scalar());
        P_ = cv::Mat(3,4, CV_64FC1, cv::Scalar());

        for(int i = 0 ; i < 9; ++i) {
            K_.at<double>(i) = info->K.at(i);
            R_.at<double>(i) = info->R.at(i);
        }
        for(int i = 0 ; i < 5 ; ++i) {
            D_.at<double>(i) = info->D.at(i);
        }
        for(int i = 0 ; i < 12 ; ++i) {
            P_.at<double>(i) = info->P.at(i);
        }
    }

    Calibration(const cv::Size &S,
                const cv::Mat  &K,
                const cv::Mat  &D,
                const cv::Mat  &R,
                const cv::Mat  &P) :
        size_(S),
        K_(K.clone()),
        D_(D.clone()),
        R_(R.clone()),
        P_(P.clone())
    {
    }

    cv::Size const & size() const
    {
        return size_;
    }

    cv::Mat const & K() const
    {
        return K_;
    }

    cv::Mat const & D() const
    {
        return D_;
    }

    cv::Mat const & R() const
    {
        return R_;
    }

    cv::Mat const & P() const
    {
        return P_;
    }

private:
    cv::Size    size_;
    cv::Mat     K_;
    cv::Mat     D_;
    cv::Mat     R_;
    cv::Mat     P_;

};
}

#endif // RAWSEEDS_ROS_CALIBRATION_HPP
