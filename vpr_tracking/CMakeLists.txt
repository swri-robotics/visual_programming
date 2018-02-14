cmake_minimum_required(VERSION 2.8.3)
project(vpr_tracking)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aruco_msgs
  eigen_conversions
  geometry_msgs
  jsk_rviz_plugins
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  urdf
  tf2_ros
  tf2_geometry_msgs
  xmlrpcpp
  vpr_msgs
  std_srvs
  visualization_msgs
  eigen_conversions
  moveit_core
  tf2_ros
  dynamic_reconfigure
  sensor_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/ToolTracker.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vpr_tracking
#  CATKIN_DEPENDS aruco_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tool_tracking_node
   src/tool_tracking_node.cpp
   src/tool.cpp
   src/tool_tracker.cpp
   src/utils.cpp
)

add_executable(path_data_manager_node
   src/path_data_manager_node.cpp
   src/path_data_manager.cpp
   src/snap_features.cpp
   src/utils.cpp
)

add_executable(test_tool_tracker
   test/test_tool_tracker.cpp
   src/tool.cpp
   src/tool_tracker.cpp
   src/utils.cpp
)

## Specify libraries to link a library or executable target against
add_dependencies(tool_tracking_node ${PROJECT_NAME}_gencfg)
target_link_libraries(tool_tracking_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

target_link_libraries(path_data_manager_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(test_tool_tracker
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
