cmake_minimum_required(VERSION 2.8.3)
project(rawseeds_ros)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
)

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(catkin REQUIRED camera_info_manager cmake_modules cv_bridge geometry_msgs image_transport rosbag roscpp sensor_msgs tf_conversions)
find_package(Eigen 3.0 REQUIRED)
find_package(OpenCV 2.3 REQUIRED)

catkin_package()

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

set(COMMON_LIBRARIES
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(camera2bag src/camera2bag.cpp)
target_link_libraries(camera2bag ${COMMON_LIBRARIES})
