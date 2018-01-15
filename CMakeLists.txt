cmake_minimum_required(VERSION 2.8.3)
project(rawseeds_ros)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(catkin REQUIRED
    camera_info_manager
    sensor_msgs
    stereo_msgs
    rosbag
    roscpp
    rospy
    pcl_conversions
    message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
    FILES
    RawseedsIMU.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime rospy
)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_stereo_camera_node
    src/nodes/stereo_camera_node.cpp
)

target_link_libraries(${PROJECT_NAME}_stereo_camera_node
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_stereo2bag
    src/tools/stereo2bag.cpp
)
target_link_libraries(${PROJECT_NAME}_stereo2bag
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_stereo2procbag
    src/tools/stereo2procbag.cpp
)
target_link_libraries(${PROJECT_NAME}_stereo2procbag
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

