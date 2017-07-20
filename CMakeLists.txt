cmake_minimum_required(VERSION 2.8.3)
project(rawseeds_ros)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(catkin REQUIRED
    camera_info_manager
    cmake_modules
    cv_bridge
    geometry_msgs
    image_transport
    rosbag
    roscpp
    sensor_msgs
    tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(COMMON_LIBRARIES
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(stereo2bag
    src/stereo2bag.cpp
)
target_link_libraries(stereo2bag
    ${COMMON_LIBRARIES}
)
