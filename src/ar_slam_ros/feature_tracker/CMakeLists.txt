cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    message_generation
    )

find_package(CUDA)
if (NOT CUDA_FOUND)
  message(STATUS "CUDA not found. Project will not be built.")
endif(NOT CUDA_FOUND)

if (WIN32)
  set(EXTRA_CXX_FLAGS "/DVERBOSE /D_CRT_SECURE_NO_WARNINGS ")
  list(APPEND CUDA_NVCC_FLAGS "-arch=sm_35;--compiler-options;-O2;-DVERBOSE") 
endif()
if (UNIX)
  if (APPLE)
    set(EXTRA_CXX_FLAGS "-DVERBOSE -msse2")
    list(APPEND CUDA_NVCC_FLAGS "-arch=sm_35;--compiler-options;-O2;-DVERBOSE") 
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    list(APPEND CUDA_NVCC_FLAGS "-lineinfo;-ccbin;/usr/bin/gcc-5;--compiler-options;-O2;-D_FORCE_INLINES;-DVERBOSE_NOT") 
  endif()
endif()

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Feature.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    CudaSift
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(cuda_sources
  CudaSift/cudaImage.cu  
  CudaSift/cudaImage.h  
  CudaSift/cudaSiftH.cu 
  CudaSift/cudaSiftH.h  
  CudaSift/matching.cu  
  CudaSift/cudaSiftD.h  
  CudaSift/cudaSift.h  
  CudaSift/cudautils.h
)

cuda_add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/BRIEF.cpp
    src/DUtils/Random.cpp
    src/DUtils/Timestamp.cpp
    ${cuda_sources}
    )

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CUDA_cudadevrt_LIBRARY} )
