project(view_controller)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

cs_add_executable(view_controller_server src/view_controller_server.cpp)


cs_install()
cs_export()
