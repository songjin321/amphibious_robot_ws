project(exposure_controller)
cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11 -Wall -Wextra)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_executable(exposure_node src/exposure_node.cpp)
target_link_libraries(exposure_node ${OpenCV_LIBRARIES})

cs_install()
cs_export()