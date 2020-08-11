#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>

#include <rosbag/bag.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>


namespace po = boost::program_options;
namespace bf = boost::filesystem;

#if CV_VERSION_MAJOR >= 4
static const auto CV_INTERPOLATION_METHOD = cv::INTER_LINEAR;
#else
static const auto CV_INTERPOLATION_METHOD = CV_INTER_LINEAR;
#endif

#if CV_VERSION_MAJOR >= 4
static const auto CV_IMAGE_LOAD_METHOD = cv::IMREAD_GRAYSCALE;
#else
static const auto CV_IMAGE_LOAD_METHOD = CV_LOAD_IMAGE_GRAYSCALE;
#endif

void readImageDirectory(const bf::path &path,
                        std::vector<std::string> &image_paths,
                        std::vector<double> &image_stamps)
{
    for(bf::directory_iterator it(path) ;
        it != bf::directory_iterator() ;
        ++it) {
        bf::path p = *it;
        if(bf::is_regular_file(p)) {
            std::string stamp = p.filename().replace_extension().string();
            std::size_t index = stamp.find_first_of("0123456789");
            stamp = stamp.substr(index);

            image_paths.emplace_back(p.string());
            image_stamps.emplace_back(boost::lexical_cast<double>(stamp));
        } else {
            std::cerr << "Recursive inputs are not supported!" << "\n";
            std::cerr << "'" << p << "' is omitted.";
        }
    }
    std::sort(image_paths.begin(), image_paths.end());
    std::sort(image_stamps.begin(), image_stamps.end());
}

bool parseCommandline(int   argc,
                      char *argv[],
                      bf::path      &path_svs_l,
                      bf::path      &path_svs_r,
                      bf::path      &path_calibration,
                      bf::path      &path_matcher,
                      bf::path      &path_bagfile,
                      bool          &debug,
                      bool          &points_only,
                      bool          &images_only,
                      std::size_t   &split_size)
{
    try {
        po::options_description desc{"Options"};
        desc.add_options()
                ("help,h", "print the help")
                ("left,l",
                 po::value<std::string>()->default_value(""),
                 "folder with image of the left camera of the stereo system.")
                ("right,r",
                 po::value<std::string>()->default_value(""),
                 "folder with image of the right camera of the stereo system.")
                ("calibration,c",
                 po::value<std::string>()->default_value(""),
                 "opencv '.yaml'-file conform the calibration format of csapex::stereo_vision.")
                ("matcher,m",
                 po::value<std::string>()->default_value(""),
                 "opencv '.yaml'-file containing the stereo matching algorithm preferences.")
                ("output,o",
                 po::value<std::string>()->default_value(""),
                 "ouput bag file.")
                ("pointsonly,p", "put only points into the bagfile")
                ("imagesonly,i", "put only images into the bagfile")
                ("split,s", "split up bag file into bags of this byte size")
                ("debug,d", "debug output");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if(argc == 1) {
            std::cout << desc << "\n";
            return false;
        }

        if(vm.count("help") == 1ul) {
            std::cout << desc << "\n";
            return false;
        } else {
            if(vm.count("left") == 0ul) {
                std::cerr << "Missing path to folder with images of the left camera of the stereo system." << "\n";
                return false;
            }
            if(vm.count("right") == 0ul) {
                std::cerr << "Missing path to folder with images of the right camera of the stereo system." << "\n";
                return false;
            }
            if(vm.count("calibration") == 0ul) {
                std::cerr << "Missing path to the calibration file of the stereo system." << "\n";
                return false;
            }
            if(vm.count("matcher") == 0ul) {
                std::cerr << "Missing path to the matcher which should be employed." << "\n";
                return false;
            }
            if(vm.count("output") == 0ul) {
                std::cerr << "Missing path for the output bagfile." << "\n";
                return false;
            }
            debug = vm.count("debug") == 1ul;
            points_only = vm.count("pointsonly") == 1ul;
            images_only = vm.count("imagesonly") == 1ul;
            points_only &= !images_only;
        }

        path_svs_l = bf::path(vm["left"].as<std::string>());
        path_svs_r = bf::path(vm["right"].as<std::string>());
        path_calibration = bf::path(vm["calibration"].as<std::string>());
        path_matcher = bf::path(vm["matcher"].as<std::string>());
        path_bagfile = bf::path(vm["output"].as<std::string>());
        split_size = vm.count("split") == 1ul ? vm["split"].as<std::size_t>() : 0ul;

        return true;
    } catch(const po::error &e) {
        std::cerr << e.what() << "\n";
    }
    return false;
}

struct Calibration {
    cv::Mat intrinsics_left_, distortion_left_;
    cv::Mat intrinsics_right_, distortion_right_;
    cv::Mat P_left_, R_left_;
    cv::Mat P_right_, R_right_;
    cv::Mat R_,T_;
    cv::Mat Q_;

    cv::Size size_;
    cv::Mat map_left_1_, map_left_2_;
    cv::Mat map_right_1_, map_right_2_;

    void copyTo(sensor_msgs::CameraInfo &left_info,
                sensor_msgs::CameraInfo &right_info)
    {
        /// left side
        left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        distortion_left_.copyTo(left_info.D);
        for(std::size_t i = 0 ; i < 9 ; ++i) {
            left_info.K[i] = distortion_left_.at<double>(static_cast<int>(i));
        }
        for(std::size_t i = 0 ; i < 9 ; ++i) {
            left_info.R[i] = R_left_.at<double>(static_cast<int>(i));
        }
        for(std::size_t i = 0 ; i < 12 ; ++i) {
            left_info.P[i] = P_left_.at<double>(static_cast<int>(i));
        }

        /// right side
        right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        distortion_left_.copyTo(right_info.D);
        for(std::size_t i = 0 ; i < 9 ; ++i) {
            right_info.K[i] = distortion_right_.at<double>(static_cast<int>(i));
        }
        for(std::size_t i = 0 ; i < 9 ; ++i) {
            right_info.R[i] = R_right_.at<double>(static_cast<int>(i));
        }
        for(std::size_t i = 0 ; i < 12 ; ++i) {
            right_info.P[i] = P_right_.at<double>(static_cast<int>(i));
        }
    }

    inline double baseline() const
    {
        return -T_.at<double>(0);
    }

    void read(cv::FileStorage &fs)
    {
        auto check_depth = [](const std::string &name,
                const cv::Mat &mat)
        {
            if(mat.type() != CV_64FC1)
                throw std::runtime_error("Matrix '" + name + "' has to be double precision!");
        };

        fs["intrinsics_left"]   >> intrinsics_left_;
        check_depth("intrinsics_left", intrinsics_left_);
        fs["distortion_left"]   >> distortion_left_;
        check_depth("distortion_left", distortion_left_);
        fs["intrinsics_right"]  >> intrinsics_right_;
        check_depth("intrinsics_right", intrinsics_right_);
        fs["distortion_right"]  >> distortion_right_;
        check_depth("distortion_right", distortion_right_);
        fs["P_left"]            >> P_left_;
        check_depth("P_left", P_left_);
        fs["R_left"]            >> R_left_;
        check_depth("R_left", R_left_);
        fs["P_right"]           >> P_right_;
        check_depth("P_right", P_right_);
        fs["R_right"]           >> R_right_;
        check_depth("R_right", R_right_);
        fs["R"]                 >> R_;
        check_depth("R", R_);
        fs["T"]                 >> T_;
        check_depth("T", T_);
        fs["Q"]                 >> Q_;
        check_depth("Q", Q_);
    }

    void undistort(const cv::Mat &left, const cv::Mat &right,
                   cv::Mat &left_undistorted, cv::Mat &right_undistorted)
    {
        if(left.rows != right.rows ||
                left.cols != right.cols) {
            throw std::runtime_error("Images must have matching size!");
        }
        cv::Size size = cv::Size(left.cols, left.rows);
        if(size_.width == 0 || size_.height == 0) {
            size_ = size;
            cv::initUndistortRectifyMap(intrinsics_left_, distortion_left_,
                                        R_left_, P_left_,
                                        size, CV_16SC2, map_left_1_, map_left_2_);
            cv::initUndistortRectifyMap(intrinsics_right_, distortion_right_,
                                        R_right_, P_right_,
                                        size, CV_16SC2, map_right_1_, map_right_2_);
        } else if(size_ != size) {
            std::cerr << "Images of size " << size << " not matching found " << size_ << " !" << "\n";
            return;
        }

        cv::remap(left, left_undistorted, map_left_1_, map_left_2_, CV_INTERPOLATION_METHOD);
        cv::remap(right, right_undistorted, map_right_1_, map_right_2_, CV_INTERPOLATION_METHOD);
    }
};

bool loadMatcher(const bf::path &path_matcher,
                 cv::Ptr<cv::StereoMatcher> &matcher)
{
    cv::FileStorage fs(path_matcher.string(), cv::FileStorage::READ);
    cv::String matcher_type;
    fs["matcher_type"] >> matcher_type;
    if(matcher_type == "BM") {
        int min_disparity;
        int num_disparitites;
        int block_size;
        int prefilter_type;
        int prefilter_size;
        int prefilter_cap;
        int texture_threshold;
        int uniqueness_ratio;
        int speckle_window_size;
        int speckle_range;
        int disparity_12_max_diff;

        fs["minDisparity"     ] >> min_disparity;
        fs["numDisparities"   ] >> num_disparitites;
        fs["blockSize"        ] >> block_size;
        fs["preFilterType"    ] >> prefilter_type;
        fs["preFilterSize"    ] >> prefilter_size;
        fs["preFilterCap"     ] >> prefilter_cap;
        fs["textureThreshold" ] >> texture_threshold;
        fs["uniquenessRatio"  ] >> uniqueness_ratio;
        fs["speckleWindowSize"] >> speckle_window_size;
        fs["speckleRange"     ] >> speckle_range;
        fs["disp12MaxDiff"    ] >> disparity_12_max_diff;

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
        matcher = bm;
    } else if(matcher_type == "SGBM") {
        int mode;
        int prefilter_cap;
        int block_size;
        int P1;
        int P2;
        int num_disparitites;
        int min_disparity;
        int uniqueness_ratio;
        int speckle_window_size;
        int speckle_range;
        int disparity_12_max_diff;

        fs["mode"             ] >> mode;
        fs["preFilterCap"     ] >> prefilter_cap;
        fs["blockSize"        ] >> block_size;
        fs["P1"               ] >> P1;
        fs["P2"               ] >> P2;
        fs["numDisparities"   ] >> num_disparitites;
        fs["minDisparity"     ] >> min_disparity;
        fs["uniquenessRatio"  ] >> uniqueness_ratio;
        fs["speckleWindowSize"] >> speckle_window_size;
        fs["speckleRange"     ] >> speckle_range;
        fs["disp12MaxDiff"    ] >> disparity_12_max_diff;

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
        matcher = sgbm;
    } else {
        std::cerr << "Unknown matcher type." << "\n";
        return false;
    }
    fs.release();
    return true;
}


int main(int argc, char *argv[])
{
    bf::path path_svs_l;
    bf::path path_svs_r;
    bf::path path_calibration;
    bf::path path_matcher;
    bf::path path_bagfile;
    bool debug = false;
    bool points_only = false;
    bool images_only = false;
    std::size_t split_size = 0ul;

    if(!parseCommandline(argc, argv, path_svs_l, path_svs_r,
                         path_calibration, path_matcher,
                         path_bagfile,
                         debug,
                         points_only,
                         images_only,
                         split_size))
        return 1;

    /// check input paths
    if(!bf::is_directory(path_svs_l)) {
        std::cerr << path_svs_l << " is not a folder!" << "\n";
        return 1;
    }
    if(bf::is_empty(path_svs_l)) {
        std::cerr << path_svs_l << " is empty!" << "\n";
        return 1;
    }
    if(!bf::is_directory(path_svs_r)) {
        std::cerr << path_svs_r << " is not a folder!" << "\n";
        return 1;
    }
    if(bf::is_empty(path_svs_r)) {
        std::cerr << path_svs_r << " is empty!" << "\n";
        return 1;
    }
    if(!bf::is_regular_file(path_calibration)) {
        std::cerr << path_calibration << " is not a file!" << "\n";
        return 1;
    }
    if(!bf::is_regular_file(path_matcher)) {
        std::cerr << path_matcher << " is not a file!" << "\n";
        return 1;
    }

    std::cout << "Current path " <<
                 bf::current_path()
              << "." << "\n";
    std::cout << "Using left input images from folder " <<
                 path_svs_l
              << "." << "\n";
    std::cout << "Using right input images from folder " <<
                 path_svs_r
              << "." << "\n";
    std::cout << "Using stereo camera calibration " <<
                 path_calibration
              << "." << "\n";
    std::cout << "Using stereo matcher configuration " <<
                 path_matcher
              << "." << "\n";
    std::cout << "Putting the data out into " <<
                 path_bagfile
              << "." << "\n";
    if(debug) {
        std::cout << "Giving you a debug output while converting." << "\n";
    }
    if(points_only) {
        std::cout << "Putting only points into the bag file." << "\n";
    }
    if(images_only) {
        std::cout << "Putting only images into the bag file." << "\n";
    }

    /// read the directories
    std::vector<std::string> images_left;
    std::vector<std::string> images_right;
    std::vector<double> images_left_stamps;
    std::vector<double> images_right_stamps;
    readImageDirectory(path_svs_l, images_left, images_left_stamps);
    readImageDirectory(path_svs_r, images_right, images_right_stamps);

    if(images_left.size() != images_right.size()) {
        std::cerr << "There must be the same amount of images in the folders." << "\n";
        return 1;
    }
    /// read the calibration
    Calibration calibration;
    cv::FileStorage fs(path_calibration.string(), cv::FileStorage::READ);
    calibration.read(fs);
    fs.release();
    /// load the matcher
    cv::Ptr<cv::StereoMatcher> matcher;
    if(!loadMatcher(path_matcher,
                    matcher)) {
        std::cerr << "Could not load matcher!" << "\n";
        return 1;
    }

    /// do the work
    const std::size_t size = images_left.size();
    const std::string left_frame_id = "svs_l";
    const std::string right_frame_id = "svs_r";
    const float       max_depth = 25.0;

    std::shared_ptr<pcl::visualization::CloudViewer> viewer;
    if(debug) {
        viewer.reset(new pcl::visualization::CloudViewer("PointCloud"));
    }

    /// CAMERA INFO
    sensor_msgs::CameraInfo     left_info_msg;
    sensor_msgs::CameraInfo     right_info_msg;
    left_info_msg.header.frame_id  = left_frame_id;
    right_info_msg.header.frame_id = right_frame_id;
    calibration.copyTo(left_info_msg, right_info_msg);

    /// IMAGE MESSAGES
    sensor_msgs::Image left_image_msg;
    sensor_msgs::Image right_image_msg;
    sensor_msgs::Image left_image_rectified_msg;
    sensor_msgs::Image right_image_rectified_msg;
    left_image_msg.header                  = left_info_msg.header;
    left_image_msg.encoding                = sensor_msgs::image_encodings::MONO8;
    left_image_msg.is_bigendian            = 0;
    right_image_msg.header                 = right_image_msg.header;
    right_image_msg.encoding               = sensor_msgs::image_encodings::MONO8;
    right_image_msg.is_bigendian           = 0;
    left_image_rectified_msg.header        = left_info_msg.header;
    left_image_rectified_msg.encoding      = sensor_msgs::image_encodings::MONO8;
    left_image_rectified_msg.is_bigendian  = 0;
    right_image_rectified_msg.header       = right_image_msg.header;
    right_image_rectified_msg.encoding     = sensor_msgs::image_encodings::MONO8;
    right_image_rectified_msg.is_bigendian = 0;

    /// DISPARITY MESSAGE
    stereo_msgs::DisparityImage disparity_image_msg;
    disparity_image_msg.header = left_info_msg.header;
    disparity_image_msg.image.header = left_info_msg.header;
    disparity_image_msg.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    disparity_image_msg.image.is_bigendian = 0ul;
    disparity_image_msg.min_disparity = matcher->getMinDisparity();
    disparity_image_msg.max_disparity = matcher->getMinDisparity() + matcher->getNumDisparities() - 1;
    disparity_image_msg.delta_d = 1.0 / 16.0;
    disparity_image_msg.f = static_cast<float>(calibration.intrinsics_right_.at<double>(0));
    disparity_image_msg.T = static_cast<float>(calibration.baseline());

    auto copyImagetoMsg = [](const cv::Mat &matrix,
            const std::size_t byte_size,
            sensor_msgs::Image &image)
    {
        image.width  = static_cast<unsigned int>(matrix.cols);
        image.height = static_cast<unsigned int>(matrix.rows);
        image.step   = static_cast<unsigned int>(matrix.cols * byte_size);
        image.data.resize(image.step * image.height);

        const uint8_t *matrix_ptr = matrix.ptr<uint8_t>();
        uint8_t *image_ptr = image.data.data();

        const std::size_t size = image.height * image.step;
        for(std::size_t i = 0 ; i < size ; ++i) {
            image_ptr[i] = matrix_ptr[i];
        }
    };

    /// bag outputfile
    rosbag::Bag bag;
    const  bf::path extension_bag_file = path_bagfile.extension();
    path_bagfile.replace_extension("");
    std::size_t split = 0;

    auto open_bag = [&bag] (const std::string &path){
#if ROS_VERSION_MAJOR >= 1 && ROS_VERSION_MINOR >= 13
        if(bag.isOpen()) {
            bag.close();
        }
#else
    bag.close();
#endif
        bag.open(path, rosbag::bagmode::Write);
    };

    if(split_size > 0) {
        open_bag(path_bagfile.string() + "_" + std::to_string(split) + extension_bag_file.string());
        ++split;
    } else {
        open_bag(path_bagfile.string() + extension_bag_file.string());
    }

//    bag.setCompression(rosbag::CompressionType::BZ2);

    std::cout << "String generation ... \n";
    std::cout << "Writing '" << size << "' images to '" << bag.getFileName() << "'.\n";

    for(std::size_t j = 0 ; j < size ; ++j) {
        if(bag.getSize() > split_size && split_size > 0) {
            open_bag(path_bagfile.string() + "_" + std::to_string(split) + extension_bag_file.string());
            ++split;
            std::cout << "Writing to '" << bag.getFileName() << "'.\n";
        }


        const double stamp_left = images_left_stamps[j];
        const double stamp_right= images_right_stamps[j];
        cv::Mat left_rectified, right_rectified;
        cv::Mat left  = cv::imread(images_left[j], CV_IMAGE_LOAD_METHOD);
        cv::Mat right = cv::imread(images_right[j], CV_IMAGE_LOAD_METHOD);
        calibration.undistort(left, right, left_rectified, right_rectified);


        cv::Mat disparity_sc, disparity_f;
        matcher->compute(left_rectified, right_rectified, disparity_sc);
        // We convert from fixed-point to float disparity and also adjust for any x-offset between
        // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
        disparity_sc.convertTo(disparity_f, CV_32FC1, 1.0/16.0, -(calibration.intrinsics_left_.at<double>(0,2) -
                                                                  calibration.intrinsics_right_.at<double>(0,2)));

        cv::Mat xyz;
        cv::reprojectImageTo3D(disparity_f, xyz, calibration.Q_, true);

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
                const float depth = std::sqrt(pxyz.dot(pxyz));
                if(depth <= max_depth) {
                    Point &p = pointcloud->at(j,i);
                    p.x = pxyz.x;
                    p.y = pxyz.y;
                    p.z = pxyz.z;
                    p.r = g;
                    p.g = g;
                    p.b = g;
                }
            }
        }

        if(debug) {
            cv::Mat display_disparity;
            cv::normalize(disparity_f, display_disparity, 0.0, 1.0, cv::NORM_MINMAX);
            cv::imshow("left", left_rectified);
            cv::imshow("right", right_rectified);
            cv::imshow("disparity", display_disparity);
            cv::waitKey(5);

            if(!viewer->wasStopped()) {
                viewer->showCloud(pointcloud);
            }
        }


        /// BAG FILE
        /// Disparity
        sensor_msgs::Image &disparity_image = disparity_image_msg.image;
        copyImagetoMsg(disparity_f, sizeof(float), disparity_image);
        /// left and right images
        copyImagetoMsg(left, sizeof(uint8_t), left_image_msg);
        copyImagetoMsg(right, sizeof(uint8_t), right_image_msg);
        /// left and right images undistorted
        copyImagetoMsg(left_rectified, sizeof(uint8_t), left_image_rectified_msg);
        copyImagetoMsg(right_rectified, sizeof(uint8_t), right_image_rectified_msg);
        /// the pointcloud
        sensor_msgs::PointCloud2 pointcloud_msg;
        pcl::toROSMsg(*pointcloud, pointcloud_msg);

        double stamp_match = std::max(stamp_left, stamp_right);
        /// time stamp disparity
        disparity_image_msg.header.stamp.fromSec(stamp_match);
        disparity_image.header.stamp.fromSec(stamp_match);
        /// time stamp images
        left_image_msg.header.stamp.fromSec(stamp_left);
        right_image_msg.header.stamp.fromSec(stamp_right);
        left_image_rectified_msg.header.stamp.fromSec(stamp_left);
        right_image_rectified_msg.header.stamp.fromSec(stamp_right);
        left_info_msg.header.stamp.fromSec(stamp_left);
        right_info_msg.header.stamp.fromSec(stamp_right);
        /// point cloud
        pointcloud_msg.header.stamp.fromSec(stamp_match);
        pointcloud_msg.header.frame_id = left_frame_id;

        /// write that out
        if(stamp_left < stamp_right) {
            if(!points_only) {
                bag.write("svs_l/mono",
                          left_image_msg.header.stamp,
                          left_image_msg);
                bag.write("svs_l/mono_rectified",
                          left_image_rectified_msg.header.stamp,
                          left_image_rectified_msg);
                bag.write("svs_l/camera_info",
                          left_info_msg.header.stamp,
                          left_info_msg);
                bag.write("svs_r/mono",
                          right_image_msg.header.stamp,
                          right_image_msg);
                bag.write("svs_r/mono_rectified",
                          right_image_rectified_msg.header.stamp,
                          right_image_rectified_msg);
                bag.write("svs_r/camera_info",
                          right_info_msg.header.stamp,
                          right_info_msg);
                bag.write("svs/disparity",
                          disparity_image_msg.header.stamp,
                          disparity_image_msg);
            }
            if(!images_only) {
                bag.write("svs/pointcloud",
                          pointcloud_msg.header.stamp,
                          pointcloud_msg);
            }
        } else {
            if(!points_only) {
                bag.write("svs_r/mono",
                          right_image_msg.header.stamp,
                          right_image_msg);
                bag.write("svs_r/mono_rectified",
                          right_image_rectified_msg.header.stamp,
                          right_image_rectified_msg);
                bag.write("svs_r/camera_info",
                          right_info_msg.header.stamp,
                          right_info_msg);
                bag.write("svs_l/mono",
                          left_image_msg.header.stamp,
                          left_image_msg);
                bag.write("svs_l/mono_rectified",
                          left_image_rectified_msg.header.stamp,
                          left_image_rectified_msg);
                bag.write("svs_l/camera_info",
                          left_info_msg.header.stamp,
                          left_info_msg);
                bag.write("svs/disparity",
                          disparity_image_msg.header.stamp,
                          disparity_image_msg);
            }
            if(!images_only) {
                bag.write("svs/pointcloud",
                          pointcloud_msg.header.stamp,
                          pointcloud_msg);
            }
        }

        std::cout << "\r" << (j + 1) / static_cast<double>(size) * 100.0 << "% done..."  << std::flush;

    }
    std::cout << "\n";
    bag.close();
    cv::destroyAllWindows();

    return 0;
}
