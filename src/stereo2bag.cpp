#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

namespace po = boost::program_options;
namespace bf = boost::filesystem;

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
            std::cerr << "Recursive inputs are not supported!" << std::endl;
            std::cerr << "'" << p << "' is omitted.";
        }
    }
    std::sort(image_paths.begin(), image_paths.end());
    std::sort(image_stamps.begin(), image_stamps.end());
}

bool parseCommandline(int   argc,
                      char *argv[],
                      bf::path &path_svs_l,
                      bf::path &path_svs_r,
                      bf::path &path_calibration,
                      bf::path &path_matcher,
                      bf::path &path_bagfile,
                      bool     &debug)
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
                ("debug,d", "debug output");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if(vm.count("help") == 1ul) {
            std::cout << desc << std::endl;
            return false;
        } else {
            if(vm.count("left") == 0ul) {
                std::cerr << "Missing path to folder with images of the left camera of the stereo system." << std::endl;
                return false;
            }
            if(vm.count("right") == 0ul) {
                std::cerr << "Missing path to folder with images of the right camera of the stereo system." << std::endl;
                return false;
            }
            if(vm.count("calibration") == 0ul) {
                std::cerr << "Missing path to the calibration file of the stereo system." << std::endl;
                return false;
            }
            if(vm.count("matcher") == 0ul) {
                std::cerr << "Missing path to the matcher which should be employed." << std::endl;
                return false;
            }
            if(vm.count("output") == 0ul) {
                std::cerr << "Missing path for the output bagfile." << std::endl;
                return false;
            }
            debug = vm.count("debug") == 1ul;
        }

        path_svs_l = bf::path(vm["left"].as<std::string>());
        path_svs_r = bf::path(vm["right"].as<std::string>());
        path_calibration = bf::path(vm["calibration"].as<std::string>());
        path_matcher = bf::path(vm["matcher"].as<std::string>());

        return true;
    } catch(const po::error &e) {
        std::cerr << e.what() << std::endl;
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

    void read(cv::FileStorage &fs)
    {
        fs["intrinsics_left"]   >> intrinsics_left_;
        fs["distortion_left"]   >> distortion_left_;
        fs["intrinsics_right"]  >> intrinsics_right_;
        fs["distortion_right"]  >> distortion_right_;
        fs["P_left"]            >> P_left_;
        fs["R_left"]            >> R_left_;
        fs["P_right"]           >> P_right_;
        fs["R_right"]           >> R_right_;
        fs["R"]                 >> R_;
        fs["T"]                 >> T_;
        fs["Q"]                 >> Q_;
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
            std::cerr << "Images of size " << size << " not matching found " << size_ << " !" << std::endl;
            return;
        }

        cv::remap(left, left_undistorted, map_left_1_, map_left_2_, CV_INTER_LINEAR);
        cv::remap(right, right_undistorted, map_right_1_, map_right_2_, CV_INTER_LINEAR);
    }
};

bool loadMatcher(const bf::path &path_matcher,
                 cv::Ptr<cv::StereoMatcher> &matcher)
{
    cv::FileStorage fs(path_matcher.string(), cv::FileStorage::READ);
    cv::String matcher_type;
    fs["matcher_type"] >> matcher_type;
    std::cerr << matcher_type << std::endl;
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
        std::cerr << "Unknown matcher type." << std::endl;
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

    if(!parseCommandline(argc, argv, path_svs_l, path_svs_r,
                         path_calibration, path_matcher,
                         path_bagfile,
                         debug))
        return 1;

    /// check input paths
    if(!bf::is_directory(path_svs_l)) {
        std::cerr << path_svs_l << " is not a folder!" << std::endl;
        return 1;
    }
    if(!bf::is_directory(path_svs_r)) {
        std::cerr << path_svs_r << " is not a folder!" << std::endl;
        return 1;
    }
    if(!bf::is_regular_file(path_calibration)) {
        std::cerr << path_calibration << " is not a file!" << std::endl;
        return 1;
    }
    if(!bf::is_regular_file(path_matcher)) {
        std::cerr << path_matcher << " is not a file!" << std::endl;
        return 1;
    }

    std::cout << "Current path " <<
                 bf::current_path()
              << "." << std::endl;
    std::cout << "Using left input images from folder " <<
                 path_svs_l
              << "." << std::endl;
    std::cout << "Using right input images from folder " <<
                 path_svs_r
              << "." << std::endl;
    std::cout << "Using stereo camera calibration " <<
                 path_calibration
              << "." << std::endl;
    std::cout << "Using stereo matcher configuration " <<
                 path_matcher
              << "." << std::endl;
    std::cout << "Putting the data out into " <<
                 path_bagfile
              << "." << std::endl;

    /// read the directories
    std::vector<std::string> images_left;
    std::vector<std::string> images_right;
    std::vector<double> images_left_stamps;
    std::vector<double> images_right_stamps;
    readImageDirectory(path_svs_l, images_left, images_left_stamps);
    readImageDirectory(path_svs_r, images_right, images_right_stamps);

    if(images_left.size() != images_right.size()) {
        std::cerr << "There must be the same amount of images in the folders." << std::endl;
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
        std::cerr << "Could not load matcher!" << std::endl;
        return 1;
    }

    /// do the work
    const std::size_t size = images_left.size();
    const std::string frame_id = "svs_l";
    const double      max_depth = 25.0;

    std::shared_ptr<pcl::visualization::CloudViewer> viewer;
    if(debug) {
        viewer.reset(new pcl::visualization::CloudViewer("PointCloud"));
    }

    for(std::size_t i = 0 ; i < size ; ++i) {
        const double stamp_left = images_left_stamps[i];
        const double stamp_right= images_right_stamps[i];
        cv::Mat left  = cv::imread(images_left[i], -1);
        cv::Mat right = cv::imread(images_right[i], -1);
        calibration.undistort(left, right, left, right);

        cv::Mat disparity_sc, disparity_f;
        matcher->compute(left, right, disparity_sc);
        disparity_sc.convertTo(disparity_f, CV_32FC1, 1./16.);

        cv::Mat xyz;
        cv::reprojectImageTo3D(disparity_f, xyz, calibration.Q_, true);

        using Point      = pcl::PointXYZRGB;
        using PointCloud = pcl::PointCloud<Point>;

        PointCloud::Ptr pointcloud(new PointCloud);
        pointcloud->points.resize(xyz.rows * xyz.cols);
        pointcloud->height = xyz.rows;
        pointcloud->width  = xyz.cols;
        for(int i = 0 ; i < xyz.rows; ++i) {
            for(int j = 0 ; j < xyz.cols ; ++j) {
                const cv::Point3f   & pxyz = xyz.at<cv::Point3f>(i,j);
                const unsigned char g  = left.at<unsigned char>(i,j);
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
            cv::imshow("left", left);
            cv::imshow("right", right);
            cv::imshow("disparity", display_disparity);
            cv::waitKey(19);

            if(!viewer->wasStopped()) {
                viewer->showCloud(pointcloud);
            }
        }

        /// let's write this to a bag file
    }
    cv::destroyAllWindows();

    return 0;
}
