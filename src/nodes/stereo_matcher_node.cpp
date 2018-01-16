#include "stereo_matcher_node.h"

#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/cudastereo.hpp>

namespace rawseeds_ros {
StereoMatcherNode::StereoMatcherNode() :
    nh_("~")
{
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
    } else if (matcher_type == "CUDA_BM") {
        int num_disparitites        = nh_.param<int>("num_disparitites"     , 64);
        int block_size              = nh_.param<int>("block_size"           , 19);
        matcher_ = cv::cuda::createStereoBM(num_disparitites, block_size);
    } else {
        std::cerr << "Unknown matcher type." << "\n";
        return false;
    }

    std::vector<double> T;
    std::vector<double> R;
    std::vector<double> S;
    std::vector<double> Q;

    nh_.getParam("T", T);
    nh_.getParam("R", R);
    nh_.getParam("S", S);
    nh_.getParam("Q", Q);

    if(T.size() != 3) {
        ROS_ERROR("Translation vector for stereo camera 'T' is required with the right size.");
        return false;
    }
    if(R.size() != 9) {
        ROS_ERROR("Rotation matrix for stereo camera 'R' is required with the right size.");
        return false;
    }
    if(S.size() != 2) {
        ROS_ERROR("Expected image size for the cameras 'S' required with the right size.");
        return false;
    }
    if(Q.size() == 16) {
        ROS_INFO_STREAM("Loading Q matrix from launch file.");
        Q_ = cv::Mat(4,4,CV_64FC1,cv::Scalar());
        for(int i = 0 ; i < 16 ; ++i) {
            Q_.at<double>(i) = Q.at(i);
        }
    }


    T_ = cv::Mat(3,1,CV_64FC1,cv::Scalar());
    for(int i = 0 ; i < 3 ; ++i) {
        T_.at<double>(i) = T.at(i);
    }
    R_ = cv::Mat(3,3, CV_64FC1,cv::Scalar());
    for(int i = 0 ; i < 9 ; ++i) {
        R_.at<double>(i) = R.at(i);
    }
    S_.width    = S.at(0);
    S_.height   = S.at(1);

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


    time_delta_ = nh_.param<double>("time_delta", 1e-5);
    debug_      = nh_.param<bool>("debug", false);

    if(debug_) {
        ROS_WARN_STREAM("Debug mode is on!");
    }

    ROS_INFO_STREAM("Setup finished quite nicely!");

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
        if(std::abs((right_img_->header.stamp - msg->header.stamp).toSec()) < time_delta_) {
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
    if(!calibration_left_) {
        ROS_WARN_STREAM("Dropping matching because of missing left calibration!");
        return;
    }
    if(!calibration_right_) {
        ROS_WARN_STREAM("Dropping matching because of missing right calibration!");
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
    cv::Mat disparity;
    matcher_->compute(left_rectified, right_rectified, disparity);

    cv::Mat disparity_f;
    // We convert from fixed-point to float disparity and also adjust for any x-offset between
    // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
    // 1./16?
    //    cv::Mat_<cv::Vec3f> XYZ(disparity32F.rows,disparity32F.cols);   // Output point cloud
    //    cv::Mat_<float> vec_tmp(4,1);
    //    for(int y=0; y<disparity32F.rows; ++y) {
    //        for(int x=0; x<disparity32F.cols; ++x) {
    //            vec_tmp(0)=x; vec_tmp(1)=y; vec_tmp(2)=disparity32F.at<float>(y,x); vec_tmp(3)=1;
    //            vec_tmp = Q*vec_tmp;
    //            vec_tmp /= vec_tmp(3);
    //            cv::Vec3f &point = XYZ.at<cv::Vec3f>(y,x);
    //            point[0] = vec_tmp(0);
    //            point[1] = vec_tmp(1);
    //            point[2] = vec_tmp(2);
    //        }
    //    }
    //        disparity.convertTo(disparity_f, CV_32FC1, 1.0 / 16.0, -(calibration_left_->K().at<double>(0,2) -
    //                                                             calibration_right_->K().at<double>(0,2)));
    disparity.convertTo(disparity_f, CV_32FC1, 1.0 / 16.0);

    cv::Mat xyz;
    cv::reprojectImageTo3D(disparity_f, xyz, Q_, true);

    using Point      = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<Point>;

    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->points.resize(static_cast<std::size_t>(xyz.rows * xyz.cols));
    pointcloud->height = static_cast<unsigned int>(xyz.rows);
    pointcloud->width  = static_cast<unsigned int>(xyz.cols);

    for(int i = 0 ; i < xyz.rows; ++i) {
        for(int j = 0 ; j < xyz.cols ; ++j) {
            const cv::Point3f   & pxyz = xyz.at<cv::Point3f>(i,j);
            const unsigned char g  = left_rectified.at<unsigned char>(i,j);
            Point &p = pointcloud->at(j,i);
            p.x = pxyz.x;
            p.y = pxyz.y;
            p.z = pxyz.z;
            p.r = g;
            p.g = g;
            p.b = g;
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

    pub_points_.publish(pointcloud_msg);
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rawseeds_ros_stereo_matcher_node");
    rawseeds_ros::StereoMatcherNode st;
    st.run();

    return 0;
}
