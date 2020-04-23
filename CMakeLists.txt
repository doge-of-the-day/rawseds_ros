cmake_minimum_required(VERSION 2.8.3)
project(rawseeds_ros)

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
    tf
)

set(USE_OMP true)
if(${USE_OMP})
    add_definitions(-DUSE_OMP)
    find_package(OpenMP  REQUIRED)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    message(STATUS "[${PROJECT_NAME}]: Compiling with OpenMP!")
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(TARGET_COMPILE_DEFINITIONS
    -std=c++11 # Or -std=c++0x
)

set(TARGET_INCLUDE_DIRS
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(TARGET_LINK_LIBS
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

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

add_executable(${PROJECT_NAME}_check_bag
    src/tools/check_bag.cpp
)
target_include_directories(${PROJECT_NAME}_check_bag
    SYSTEM PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}_check_bag
    PRIVATE
        include/
)
target_link_libraries(${PROJECT_NAME}_check_bag
    PRIVATE
        ${TARGET_LINK_LIBS}
)

add_executable(${PROJECT_NAME}_stereo_matcher_node
    src/nodes/undistortion.hpp
    src/nodes/calibration.hpp
    src/nodes/stereo_matcher_node.cpp
    src/nodes/disparity_filters.hpp
    src/nodes/disparity_filters.cpp
    src/nodes/fgs_filter.hpp
    src/nodes/fgs_filter.cpp
)
target_include_directories(${PROJECT_NAME}_stereo_matcher_node
    SYSTEM PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}_stereo_matcher_node
    PRIVATE
        include/
)
target_link_libraries(${PROJECT_NAME}_stereo_matcher_node
    PRIVATE
        ${TARGET_LINK_LIBS}
)

find_package(cslibs_indexed_storage QUIET)
if(${cslibs_indexed_storage_FOUND})
    add_executable(${PROJECT_NAME}_pointcloud_filter_node
        src/nodes/pointcloud_filter_node.cpp
        src/nodes/voxel.hpp
    )
    target_include_directories(${PROJECT_NAME}_pointcloud_filter_node
        SYSTEM PRIVATE
            ${TARGET_INCLUDE_DIRS}
    )
    target_include_directories(${PROJECT_NAME}_steroe_matcher_node
    PRIVATE
        ${cslibs_indexed_storage_INCLUDE_DIRS}
        include/
    )
    target_link_libraries(${PROJECT_NAME}_pointcloud_filter_node
        PRIVATE
            ${TARGET_LINK_LIBS}
            ${cslibs_indexed_storage_LIBRARIES}
    )
endif()


if(${OpenCV_CUDA_VERSION})
    message(STATUS "[${PROJECT_NAME}]: Compiling with OpenCV cuda!")
    add_executable(${PROJECT_NAME}_stereo_matcher_cuda_node
        src/nodes/undistortion.hpp
        src/nodes/calibration.hpp
        src/nodes/cuda_matchers.hpp
        src/nodes/stereo_matcher_cuda_node.cpp
    )
    target_include_directories(${PROJECT_NAME}_stereo_matcher_cuda_node
        SYSTEM PRIVATE
            ${TARGET_INCLUDE_DIRS}
    )
    target_include_directories(${PROJECT_NAME}_stereo_matcher_cuda_node
        PRIVATE
            include/
    )
    target_link_libraries(${PROJECT_NAME}_stereo_matcher_cuda_node
        PRIVATE
            ${TARGET_LINK_LIBS}
    )
endif()

add_executable(${PROJECT_NAME}_stereo2bag
    src/tools/stereo2bag.cpp
)
target_include_directories(${PROJECT_NAME}_stereo2bag
    SYSTEM PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}_stereo2bag
    PRIVATE
        include/
)
target_link_libraries(${PROJECT_NAME}_stereo2bag
    PRIVATE
        ${TARGET_LINK_LIBS}
)


add_executable(${PROJECT_NAME}_stereo2procbag
    src/tools/stereo2procbag.cpp
)
target_include_directories(${PROJECT_NAME}_stereo2procbag
    SYSTEM PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}_stereo2procbag
    PRIVATE
        include/
)
target_link_libraries(${PROJECT_NAME}_stereo2procbag
    PRIVATE
        ${TARGET_LINK_LIBS}
)
