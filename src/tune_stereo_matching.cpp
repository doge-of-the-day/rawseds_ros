#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <map>
#include <vector>

void readDirectory(const boost::filesystem::path &path,
                   std::map<std::string, std::string> &images)
{
    boost::filesystem::directory_iterator it_end;
    for(boost::filesystem::directory_iterator it(path) ;
        it != it_end;
        ++it) {
        const boost::filesystem::path &p = it->path();

        std::string filename = p.filename().string();
        std::string extension = p.extension().string();
        int npos =filename.size() - extension.size();
        std::string fileid = filename.substr(6, npos - 10);
        std::string path = p.string();
        images.emplace(std::make_pair(fileid, path));
    }
}

void runStereoMatcher(const cv::Mat &right,
                      const cv::Mat &left,
                      cv::Mat &disparity)
{

}


void buttonBM(int state, void *userdata)
{
    *(static_cast<bool*>(userdata)) = false;
}

void buttonSGBM(int state, void *userdata)
{
    *(static_cast<bool*>(userdata)) = true;
}

int main(int argc, char *argv[])
{
    if(argc != 3) {
        std::cerr << "tune_stereo_matching <SVS_R> <SVS_L>" << std::endl;
        return -1;
    }

    std::map<std::string, std::string> paths_left;
    std::map<std::string, std::string> paths_right;
    {
        boost::filesystem::path path_right(argv[1]);
        boost::filesystem::path path_left(argv[2]);

        if(!boost::filesystem::is_directory(path_right)) {
            std::cerr << "<SVS_R> must be a path to a directory!" << std::endl;
            return -2;
        }
        if(!boost::filesystem::is_directory(path_left)) {
            std::cerr << "<SVS_L> must be a path to a directory!" << std::endl;
            return -2;
        }

        readDirectory(path_left, paths_left);
        readDirectory(path_right, paths_right);

        if(paths_left.size() != paths_right.size()) {
            std::cerr << "<SVS_R> and <SVS_L> should contain the same amount of images!" << std::endl;
        }
    }

    if(paths_left.size() == 0) {
        std::cerr << "<SVS_R> is empty!" << std::endl;
        return -3;
    }
    if(paths_left.size() == 0) {
        std::cerr << "<SVS_L> is empty!" << std::endl;
        return -3;
    }


    const int ESC = 27;
    const int ARR_LEFT  = 81;
    const int ARR_RIGHT = 83;
    const int SPACE = 32;

    std::map<std::string, std::string>::const_iterator it_right = paths_right.begin();
    auto increment = [&paths_right, &it_right] () {
        ++it_right;
        if(it_right == paths_right.end()) {
            it_right = paths_right.begin();
        }
    };
    auto decrement = [&paths_right, &it_right] () {
        if(it_right == paths_right.begin()) {
            it_right = paths_right.end();
        }
        --it_right;
    };

    //// stereo matching and parameters
    bool                    use_sgbm = false;
    cv::Ptr<cv::StereoBM>   stereo_bm;
    cv::Ptr<cv::StereoSGBM> stereo_sgbm;

    int blockSize         = 21;
    int numDisparities    = 0;
    int minDisparity      = 0;
    int P1                = 0;
    int P2                = 0;
    int disp12MaxDiff     = 0;
    int preFilterCap      = 0;
    int uniquenessRatio   = 0;
    int speckleWindowSize = 0;
    int speckleRange      = 0;
    int mode              = cv::StereoSGBM::MODE_SGBM;





    //// images and buffers
    cv::Mat left;
    cv::Mat right;
    cv::Mat left_gray;
    cv::Mat right_gray;
    cv::Mat display;

    ////
    bool    update = false;
    bool    auto_increment = false;

    cv::namedWindow("display");
    cv::createButton("BM",   buttonBM,   &use_sgbm,CV_RADIOBOX, 1);
    cv::createButton("SGBM", buttonSGBM, &use_sgbm,CV_RADIOBOX, 0);

    int key = 0;
    while(key != ESC) {
        if(left.empty() || right.empty() || update) {
            const std::string id = it_right->first;
            const std::string &path_right = it_right->second;
            const std::string &path_left = paths_left[id];
            if(path_right != "" && path_left != "") {
                right = cv::imread(path_right);
                left = cv::imread(path_left);
                cv::cvtColor(right, right_gray, CV_BGR2GRAY);
                cv::cvtColor(left, left_gray, CV_BGR2GRAY);
                update = false;
            } else {
                std::cerr << id << " : paths missing" << std::endl;
            }
        }

        if(!right.empty() && !left.empty()) {
            display = cv::Mat(left.rows, left.cols + right.cols, left.type(), cv::Scalar());
            cv::Mat left_roi  = cv::Mat(display, cv::Rect(0,0, left.cols, left.rows));
            cv::Mat right_roi = cv::Mat(display, cv::Rect(left.cols, 0, right.cols, right.rows));
            left.copyTo(left_roi);
            right.copyTo(right_roi);
            cv::imshow("display", display);

            /// run the stereo matcher here
            cv::Mat left_gray;
            cv::Mat right_gray;

            if(use_sgbm) {
                cv::Mat disparity;
                stereo_sgbm = cv::StereoSGBM::create(minDisparity,
                                                     numDisparities,
                                                     blockSize,
                                                     P1,
                                                     P2,
                                                     disp12MaxDiff,
                                                     preFilterCap,
                                                     uniquenessRatio,
                                                     speckleWindowSize,
                                                     speckleRange,
                                                     mode);
                stereo_sgbm->compute(left_gray, right_gray, disparity);
                cv::imshow("disparity", disparity);
            } else {
                cv::Mat disparity;
                stereo_bm = cv::StereoBM::create(numDisparities, blockSize);
                stereo_bm->compute(left_gray, right_gray, disparity);
                cv::imshow("disparity", disparity);
            }
        }

        key = cv::waitKey(19) & 0xFF;
        if(auto_increment) {
            increment();
            update = true;
        }

        switch(key) {
        case ARR_LEFT:
            decrement();
            update = true;
            break;
        case ARR_RIGHT:
            increment();
            update = true;
            break;
        case SPACE:
            auto_increment = !auto_increment;
            break;
        default:
            break;
        }

    }

    return 0;
}
