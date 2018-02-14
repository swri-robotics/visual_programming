cmake_minimum_required(VERSION 2.8.3)
project(vpr_support)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vpr_support
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
