#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <rosbag/bag.h>


namespace po = boost::program_options;
namespace bf = boost::filesystem;

bool parseCommandline(int   argc,
                      char *argv[],
                      bf::path      &in_path_bag,
                      bf::path      &out_path_bag)
{
    try {
        po::options_description desc{"Options"};
        desc.add_options()
                ("help,h", "print the help")
                ("in,i",
                 po::value<std::string>()->default_value(""),
                 "input bag file.")
                ("out,o",
                 po::value<std::string>()->default_value(""),
                 "output bag file [optional, if not only debug output is given].");

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
            if(vm.count("in") == 0ul) {
                std::cerr << "Missing path to folder with images of the left camera of the stereo system." << "\n";
                return false;
            } else {
              in_path_bag = bf::path(vm["in"].as<std::string>());
            }
            if(vm.count("out") == 1ul) {
              out_path_bag = bf::path(vm["out"].as<std::string>());
            }
        }
        return true;
    } catch(const po::error &e) {
        std::cerr << e.what() << "\n";
    }
    return false;
}

int main(int argc, char *argv[])
{
    bf::path in_path_bag;
    bf::path out_path_bag;

    if(!parseCommandline(argc, argv, in_path_bag, out_path_bag))
        return 1;

    return 0;
}
