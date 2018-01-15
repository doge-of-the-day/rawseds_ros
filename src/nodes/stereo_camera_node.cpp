#include "stereo_camera_node.h"

namespace rawseeds_ros {
StereoCameraNode::StereoCameraNode() :
    nh_("~")
{

}

void StereoCameraNode::run()
{

}

bool StereoCameraNode::setup()
{
    ROS_INFO_STREAM("Initializing the matcher.");
    const std::string matcher_type = nh_.param<std::string>("matcher_type", "BM");
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

    ROS_INFO_STREAM("Setting up the subscribers");
    const std::string topic_left_img    = nh_.param<std::string>("topic_left_img",   "/svs_l/image_raw");
    const std::string topic_left_info   = nh_.param<std::string>("topic_left_info",  "/svs_l/camera_info");
    const std::string topic_right_img   = nh_.param<std::string>("topic_right_img",  "/svs_r/image_raw");
    const std::string topic_right_info  = nh_.param<std::string>("topic_right_info", "/svs_r/camera_info");

}


void StereoCameraNode::left(const sensor_msgs::ImageConstPtr &msg)
{

}
void StereoCameraNode::left(const sensor_msgs::CameraInfoConstPtr &msg)
{

}

void StereoCameraNode::right(const sensor_msgs::ImageConstPtr &msg)
{

}
void StereoCameraNode::right(const sensor_msgs::CameraInfoConstPtr &msg)
{

}
}

int main(int argc, char *argv[])
{

    return 0;
}
