cmake_minimum_required(VERSION 3.12.0)
project(vins_estimator)

SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_model
    vikit_common 
    vikit_ros
    svo
    )

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

catkin_package()

add_library(vins_lib
src/ORBextractor.cpp
src/parameters.cpp
src/PatchMatcher.cpp
src/factor/pose_local_parameterization.cpp
src/factor/projection_factor.cpp
src/factor/projection_td_factor.cpp
src/factor/marginalization_factor.cpp
src/utility/utility.cpp
src/utility/visualization.cpp
src/utility/CameraPoseVisualization.cpp
src/initial/solve_5pts.cpp
src/initial/initial_aligment.cpp
src/initial/initial_sfm.cpp
src/initial/initial_ex_rotation.cpp
)

SET(Sophus_LIBRARIES libSophus.so)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Sophus_LIBRARIES}) 

add_executable(vins_estimator
    src/estimator_node.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    )
target_link_libraries(vins_estimator vins_lib) 

ADD_EXECUTABLE(test_matcher test/test_matcher.cpp)
TARGET_LINK_LIBRARIES(test_matcher vins_lib)

