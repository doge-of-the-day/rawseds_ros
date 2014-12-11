#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <opencv2/opencv.hpp>

namespace po = boost::program_options;

using namespace std;

int main(int ac, char** av)
{
    // Declare the supported options.
    po::options_description desc("Program options");
    desc.add_options()
            ("help", "produce help message")
            ("input", po::value<std::string>(), "input base name")
            ("output", po::value<std::string>(), "output file name")
            ("topic", po::value<std::string>(), "topic name")
            ("frame_id", po::value<std::string>(), "frame_id")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);

    if (!vm.count("input") || !vm.count("output") || !vm.count("topic") || !vm.count("frame_id")) {
        cout << "missing option" << endl;
        cout << desc << endl;
        return EXIT_FAILURE;
    }

    std::string input    = vm["input"].as<std::string>();
    std::string output   = vm["output"].as<std::string>();
    std::string topic    = vm["topic"].as<std::string>();
    std::string frame_id = vm["frame_id"].as<std::string>();

    cout << "Parameters:" << endl
         << "input: " << input << endl
         << "output: " << output << endl
         << "topic: " << topic << endl
         << "frame_id: " << frame_id << endl;

    string ts_file = input + ".csv";
    string files_file = input + ".lst";

    std::ifstream ts_stream(ts_file.c_str());
    std::ifstream files_stream(files_file.c_str());

    if (!ts_stream.is_open()) {
        cerr << "could not open: " << ts_file << endl;
        return EXIT_FAILURE;
    }

    if (!files_stream.is_open()) {
        cerr << "could not open: " << files_file << endl;
        return EXIT_FAILURE;
    }

    rosbag::Bag bag;
    bag.open(output, rosbag::bagmode::Write);

    string ts, file;
    while (getline(ts_stream, ts)) {
        getline(files_stream, file);
        cout << "ts: " << ts << " file: " << file << endl;

        cv_bridge::CvImage cvImage;
        cvImage.header.frame_id = frame_id;
        cvImage.header.stamp.fromSec(boost::lexical_cast<double>(ts));
        cvImage.encoding = "mono8";
        cvImage.image = cv::imread(file, cv::IMREAD_GRAYSCALE);

        bag.write(topic, cvImage.header.stamp, cvImage.toImageMsg());
    }
}
