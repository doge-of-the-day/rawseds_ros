#include "stereo_matcher_node.h"

#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <omp.h>
#include <thread>

namespace rawseeds_ros {
StereoMatcherNode::StereoMatcherNode() :
    nh_("~")
{
#ifdef USE_OMP
    omp_set_dynamic(0);
    omp_set_num_threads(2);
#endif
}

void StereoMatcherNode::run()
{
    if(!setup()) {
        ROS_ERROR_STREAM("Could not set up the matching node!");
        return;
    }
    ros::spin();
}

bool StereoMatcherNode::setup()
{
    ROS_INFO_STREAM("Initializing the matcher.");
    const std::string matcher_type = nh_.param<std::string>("matcher_type", "SGBM");
    if(matcher_type == "BM") {
        int min_disparity           = nh_.param<int>("min_disparity"        , 1);
        int num_disparitites        = nh_.param<int>("num_disparitites"     , 48);
        int block_size              = nh_.param<int>("block_size"           , 51);
        int prefilter_type          = nh_.param<int>("prefilter_type"       , 1);
        int prefilter_size          = nh_.param<int>("prefilter_size"       , 15);
        int prefilter_cap           = nh_.param<int>("prefilter_cap"        , 20);
        int texture_threshold       = nh_.param<int>("texture_threshold"    , 0);
        int uniqueness_ratio        = nh_.param<int>("uniqueness_ratio"     , 3);
        int speckle_window_size     = nh_.param<int>("speckle_window_size"  , 716);
        int speckle_range           = nh_.param<int>("speckle_range"        , 491);
        int disparity_12_max_diff   = nh_.param<int>("disparity_12_max_diff", 1);

        auto bm = cv::StereoBM::create(num_disparitites, block_size);
        bm->setPreFilterCap(prefilter_cap);
        bm->setPreFilterType(prefilter_type);
        bm->setMinDisparity(min_disparity);
        bm->setNumDisparities(num_disparitites);
        bm->setTextureThreshold(texture_threshold);
        bm->setUniquenessRatio(uniqueness_ratio);
        bm->setSpeckleWindowSize(speckle_window_size);
        bm->setSpeckleRange(speckle_range);
        bm->setDisp12MaxDiff(disparity_12_max_diff);
        bm->setPreFilterSize(prefilter_size);
        matcher_ = bm;
    } else if(matcher_type == "SGBM") {
        int mode                    = nh_.param<int>("mode"                 , 2);
        int prefilter_cap           = nh_.param<int>("prefilter_cap"        , 63);
        int block_size              = nh_.param<int>("block_size"           , 7);
        int P1                      = nh_.param<int>("P1"                   , 392);
        int P2                      = nh_.param<int>("P2"                   , 1568);
        int num_disparitites        = nh_.param<int>("num_disparitites"     , 64);
        int min_disparity           = nh_.param<int>("min_disparity"        , 0);
        int uniqueness_ratio        = nh_.param<int>("uniqueness_ratio"     , 32);
        int speckle_window_size     = nh_.param<int>("speckle_window_size"  , 152);
        int speckle_range           = nh_.param<int>("speckle_range"        , 200);
        int disparity_12_max_diff   = nh_.param<int>("disparity_12_max_diff", -1);

        auto sgbm = cv::StereoSGBM::create(min_disparity,
                                           num_disparitites,
                                           block_size,
                                           P1,
                                           P2,
                                           disparity_12_max_diff,
                                           prefilter_cap,
                                           uniqueness_ratio,
                                           speckle_window_size,
                                           speckle_range,
                                           mode);

        matcher_ = sgbm;
    } else {
        std::cerr << "Unknown matcher type." << "\n";
        return false;
    }

    /// READ ESSENTIAL PARAMETERS
    std::vector<double> buffer;
    nh_.getParam("T", buffer);
    if(buffer.size() != 3) {
        ROS_ERROR("Translation vector for stereo camera 'T' is required with the right size.");
        return false;
    }
    T_ = cv::Mat(3,1,CV_64FC1,cv::Scalar());
    for(int i = 0 ; i < 3 ; ++i) {
        T_.at<double>(i) = buffer.at(i);
    }
    buffer.clear();
    nh_.getParam("R", buffer);
    if(buffer.size() != 9) {
        ROS_ERROR("Rotation matrix for stereo camera 'R' is required with the right size.");
        return false;
    }
    R_ = cv::Mat(3,3, CV_64FC1,cv::Scalar());
    for(int i = 0 ; i < 9 ; ++i) {
        R_.at<double>(i) = buffer.at(i);
    }
    buffer.clear();
    nh_.getParam("S", buffer);
    if(buffer.size() != 2) {
        ROS_ERROR("Expected image size for the cameras 'S' required with the right size.");
        return false;
    }
    S_.width    = buffer.at(0);
    S_.height   = buffer.at(1);

    /// READ OPTIONALS
    buffer.clear();
    nh_.getParam("Q", buffer);
    if(buffer.size() == 16) {
        ROS_INFO_STREAM("Loading Q matrix from launch file.");
        Q_ = cv::Mat(4,4,CV_64FC1,cv::Scalar());
        for(int i = 0 ; i < 16 ; ++i) {
            Q_.at<double>(i) = buffer.at(i);
        }
    }

    auto try_read_calibration = [&buffer, this] (const std::string &suffix,
            Calibration::Ptr &calib,
            Undistortion::Ptr &undist)
    {
        cv::Mat K = cv::Mat(3,3, CV_64FC1, cv::Scalar());
        cv::Mat D = cv::Mat(5,1, CV_64FC1, cv::Scalar());
        cv::Mat R = cv::Mat(3,3, CV_64FC1, cv::Scalar());
        cv::Mat P = cv::Mat(3,4, CV_64FC1, cv::Scalar());

        buffer.clear();
        nh_.getParam("K"+suffix, buffer);
        if(buffer.size() != 9)
            return;
        for(int i = 0 ; i < 9 ; ++i) {
            K.at<double>(i) = buffer.at(i);
        }

        buffer.clear();
        nh_.getParam("D"+suffix, buffer);
        if(buffer.size() != 5)
            return;

        for(int i = 0 ; i < 5 ; ++i) {
            D.at<double>(i) = buffer.at(i);
        }

        buffer.clear();
        nh_.getParam("R"+suffix, buffer);
        if(buffer.size() != 9)
            return;
        for(int i = 0 ; i < 9 ; ++i) {
            R.at<double>(i) = buffer.at(i);
        }

        buffer.clear();
        nh_.getParam("P"+suffix, buffer);
        if(buffer.size() != 12)
            return;
        for(int i = 0 ; i < 12 ; ++i) {
            P.at<double>(i) = buffer.at(i);
        }

        calib.reset(new Calibration(S_, K, D, R, P));
        undist.reset(new Undistortion(S_, K, D, R, P));
    };

    try_read_calibration("_left", calibration_left_, undistortion_left_);
    try_read_calibration("_right", calibration_right_, undistortion_right_);



    ROS_INFO_STREAM("Setting up the subscribers");
    const int queue_size                = nh_.param<int>("queue_size", 1);
    const std::string topic_left_img    = nh_.param<std::string>("topic_left_img",   "/svs_l/image_raw");
    const std::string topic_left_info   = nh_.param<std::string>("topic_left_info",  "/svs_l/camera_info");
    const std::string topic_right_img   = nh_.param<std::string>("topic_right_img",  "/svs_r/image_raw");
    const std::string topic_right_info  = nh_.param<std::string>("topic_right_info", "/svs_r/camera_info");
    const std::string topic_points      = nh_.param<std::string>("topic_points",     "/svs_l/points");
    sub_left_img_   = nh_.subscribe<sensor_msgs::Image>(topic_left_img, queue_size, &StereoMatcherNode::left, this);
    sub_left_info_  = nh_.subscribe<sensor_msgs::CameraInfo>(topic_left_info, queue_size, &StereoMatcherNode::left, this);
    sub_right_img_  = nh_.subscribe<sensor_msgs::Image>(topic_right_img, queue_size, &StereoMatcherNode::right, this);
    sub_right_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(topic_right_info, queue_size, &StereoMatcherNode::right, this);
    pub_points_     = nh_.advertise<sensor_msgs::PointCloud2>(topic_points, 1);

    max_depth_      = nh_.param<double>("max_depth", 25.0);

    time_delta_ = nh_.param<double>("time_delta", 1e-8);
    debug_      = nh_.param<bool>("debug", false);

    if(debug_) {
        ROS_WARN_STREAM("Debug mode is on!");
    }


    wls_sigma_      = nh_.param<double>("wls_sigma", 0.5);
    wls_lambda_     = nh_.param<double>("wls_lambda", 8000.0);
    use_wls_filter_ = nh_.param<double>("use_wls_filter", false);
    if(use_wls_filter_) {
        wls_filter_ = cv::ximgproc::createDisparityWLSFilter(matcher_);
        wls_filter_->setSigmaColor(wls_sigma_);
        wls_filter_->setLambda(wls_lambda_);
        wls_right_matcher_ = cv::ximgproc::createRightMatcher(matcher_);
    }

    ROS_INFO_STREAM("Setup finished quite nicely!");

    const double pub_rate = nh_.param<double>("rate", 0.0);
    pub_period_    = ros::Duration(pub_rate > 0.0 ? 1.0 / pub_rate : 0.0);
    pub_last_time_ = ros::Time::now();

    return true;
}


void StereoMatcherNode::left(const sensor_msgs::ImageConstPtr &msg)
{
    if(msg->encoding != sensor_msgs::image_encodings::MONO8) {
        ROS_ERROR_STREAM("Only MONO8 images are supported!");
        ros::shutdown();
    }
    if(msg->height != S_.height) {
        ROS_ERROR_STREAM("Image height does not match expeted height!");
        ros::shutdown();
    }
    if(msg->width != S_.width) {
        ROS_ERROR_STREAM("Image height does not match expeted height!");
        ros::shutdown();
    }

    left_img_   = msg;

    if(right_img_) {
        if(std::abs((right_img_->header.stamp - msg->header.stamp).toSec()) < time_delta_) {
            match();
        }
    }
}
void StereoMatcherNode::left(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if(!calibration_left_) {
        if(msg->height != S_.height ||
                msg->width != S_.width) {
            ROS_ERROR_STREAM("Left camera info contains size which is not matching: ["
                             << S_.width << "," << S_.height
                             << "] vs. ["
                             << msg->width << "," << msg->height << "]");
            ros::shutdown();
        }

        calibration_left_.reset(new Calibration(msg));
        if(debug_) {
            ROS_INFO_STREAM("Left calibration: \n" <<
                            "K: \n" << calibration_left_->K() << "\n" <<
                            "D: \n" << calibration_left_->D() << "\n" <<
                            "P: \n" << calibration_left_->P() << "\n" <<
                            "R: \n" << calibration_left_->R() << "\n");
        }

        undistortion_left_.reset(new Undistortion(calibration_left_->size(),
                                                  calibration_left_->K(),
                                                  calibration_left_->D(),
                                                  calibration_left_->R(),
                                                  calibration_left_->P()));
        if(calibration_right_) {
            stereoRectify();
        }
    }
}

void StereoMatcherNode::right(const sensor_msgs::ImageConstPtr &msg)
{
    if(msg->encoding != sensor_msgs::image_encodings::MONO8) {
        ROS_ERROR_STREAM("Only MONO8 images are supported!");
        ros::shutdown();
    }
    if(msg->height != S_.height) {
        ROS_ERROR_STREAM("Image height does not match expeted height!");
        ros::shutdown();
    }
    if(msg->width != S_.width) {
        ROS_ERROR_STREAM("Image height does not match expeted height!");
        ros::shutdown();
    }

    right_img_   = msg;

    if(right_img_) {
        if(std::abs((left_img_->header.stamp - msg->header.stamp).toSec()) < time_delta_) {
            match();
        }
    }

}

void StereoMatcherNode::right(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if(!calibration_right_) {
        if(msg->height != S_.height ||
                msg->width != S_.width) {
            ROS_ERROR_STREAM("Right camera info contains size which is not matching: ["
                             << S_.width << "," << S_.height
                             << "] vs. ["
                             << msg->width << "," << msg->height << "]");
            ros::shutdown();
        }

        calibration_right_.reset(new Calibration(msg));
        if(debug_) {
            ROS_INFO_STREAM("Right calibration: \n" <<
                            "K: \n" << calibration_right_->K() << "\n" <<
                            "D: \n" << calibration_right_->D() << "\n" <<
                            "P: \n" << calibration_right_->P() << "\n" <<
                            "R: \n" << calibration_right_->R() << "\n");
        }

        undistortion_right_.reset(new Undistortion(calibration_right_->size(),
                                                   calibration_right_->K(),
                                                   calibration_right_->D(),
                                                   calibration_right_->R(),
                                                   calibration_right_->P()));
        if(calibration_left_) {
            stereoRectify();
        }
    }

}

void StereoMatcherNode::stereoRectify()
{
    if(!Q_.empty())
        return;

    cv::Mat R1, R2, P1, P2;
    cv::stereoRectify(calibration_left_->K(), calibration_left_->D(),
                      calibration_right_->K(), calibration_right_->D(),
                      S_, R_, T_, R1, R2, P1, P2, Q_);
}

void StereoMatcherNode::match()
{
    ros::Time now = ros::Time::now();
    if(pub_last_time_ + pub_period_ > now)
        return;

    if(!calibration_left_) {
        ROS_WARN_STREAM("Dropping matching because of missing left calibration!");
        return;
    }
    if(!calibration_right_) {
        ROS_WARN_STREAM("Dropping matching because of missing right calibration!");
        return;
    }
    if(!left_img_) {
        ROS_WARN_STREAM("Dropping matching because of missing left image!");
        return;
    }
    if(!right_img_) {
        ROS_WARN_STREAM("Dropping matching because of missing right image!");
        return;
    }
    if(!undistortion_left_) {
        ROS_WARN_STREAM("Dropping matching because of missing left undistortion!");
        return;
    }
    if(!undistortion_right_) {
        ROS_WARN_STREAM("Dropping matching because of missing right undistortion!");
        return;
    }

    cv::Mat left(S_.height, S_.width, CV_8UC1, cv::Scalar());
    cv::Mat right(S_.height, S_.width, CV_8UC1, cv::Scalar());
    for(int i = 0 ; i < S_.height ; ++i) {
        for(int j = 0 ; j < S_.width ; ++j) {
            left.at<uchar>(i,j)  = left_img_->data[i * S_.width + j];
            right.at<uchar>(i,j) = right_img_->data[i * S_.width + j];
        }
    }

    cv::Mat left_rectified, right_rectified;
    undistortion_left_->apply(left, left_rectified);
    undistortion_right_->apply(right, right_rectified);

    cv::Mat disparity_f;
    if(use_wls_filter_) {
        cv::Mat disparity, disparity_left, disparity_right;
        auto match_left = [&left_rectified, &right_rectified, disparity_left, this]() {
            matcher_->compute(left_rectified, right_rectified, disparity_left);
        };
        auto match_right = [&left_rectified, &right_rectified, disparity_right, this]() {
            wls_right_matcher_->compute(right_rectified, left_rectified, disparity_right);
        };

        std::thread tl = std::thread(match_left);
        std::thread tr = std::thread(match_right);
        tl.join();
        tr.join();

        wls_filter_->filter(disparity_left, disparity, disparity_right);
        disparity.convertTo(disparity_f, CV_32FC1, 1.0 / 16.0);
    } else {
        cv::Mat disparity;
        matcher_->compute(left_rectified, right_rectified, disparity);
        disparity.convertTo(disparity_f, CV_32FC1, 1.0 / 16.0);
    }


    cv::Mat xyz(disparity_f.rows, disparity_f.cols, CV_32FC3, cv::Scalar());
    cv::reprojectImageTo3D(disparity_f, xyz, Q_, true);

    using Point      = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<Point>;

    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->header.frame_id = left_img_->header.frame_id;
    pcl_conversions::toPCL(now, pointcloud->header.stamp);
    pointcloud->points.resize(static_cast<std::size_t>(xyz.rows * xyz.cols));
    pointcloud->height = static_cast<unsigned int>(xyz.rows);
    pointcloud->width  = static_cast<unsigned int>(xyz.cols);
    pointcloud->is_dense = false;

#pragma omp parallel for
    for(int i = 0 ; i < xyz.rows; ++i) {
        for(int j = 0 ; j < xyz.cols ; ++j) {
            const cv::Point3f   & pxyz = xyz.at<cv::Point3f>(i,j);

            const unsigned char g  = left_rectified.at<unsigned char>(i,j);
            Point &p = pointcloud->at(j,i);
            if(pxyz.z >= max_depth_) {
                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            } else {
                p.x = pxyz.x;
                p.y = pxyz.y;
                p.z = pxyz.z;
                p.r = g;
                p.g = g;
                p.b = g;
            }
        }
    }

    if(debug_) {
        cv::normalize(disparity_f, disparity_f, 0.0, 1.0, cv::NORM_MINMAX);
        cv::imshow("disparity", disparity_f);
        cv::imshow("left_rectified", left_rectified);
        cv::imshow("right_rectified", right_rectified);
        cv::waitKey(5);
    }

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*pointcloud, pointcloud_msg);
    pointcloud_msg.header = left_img_->header;
    pointcloud_msg.header.stamp = now;

    pub_points_.publish(pointcloud_msg);
    pub_last_time_ = now;
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rawseeds_ros_stereo_matcher_node");
    rawseeds_ros::StereoMatcherNode st;
    st.run();

    return 0;
}
