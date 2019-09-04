cmake_minimum_required(VERSION 2.8.3)
project(ar_slam_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
    )

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

# add ar_slam, generater corresponds library
add_subdirectory(ar_slam)

# find package
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
## glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS}) 
## eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS}) 
## opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) 

# add include path
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ar_slam/estimator/include
    ar_slam/backend/include
    ar_slam/base/include
    ar_slam/common/include
    ar_slam/frontend/include
)                        

# compile the actual application
add_executable(ar_slam_node
    src/Subscriber.cpp
    src/Publisher.cpp
    src/ar_slam_node.cpp
    )

target_link_libraries(ar_slam_node 
${catkin_LIBRARIES}
${GLOG_LIBRARIES}
${OpenCV_LIBS}
arslam_common
arslam_estimator
) 
