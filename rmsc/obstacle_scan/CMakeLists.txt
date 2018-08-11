cmake_minimum_required(VERSION 2.8.3)
project(obstacle_scan)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  laser_geometry
  genmsg 
  actionlib
  actionlib_msgs 
  roslib
)

find_package(OpenCV REQUIRED)

# vvvvvvvvvvvvvvvvvvvvvvvvvv
find_package(Protobuf REQUIRED)
protobuf_generate_cpp(proto/arguments.pb.cc proto/arguments.pb.h proto/arguments.proto)

#AAAAAAAAAAAAAAAAAAAAAAAAAAAAA

#vvvvvvvvvvvvvvvvvvvvvvvvv
add_action_files(DIRECTORY action FILES ObstacleScan.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
#AAAAAAAAAAAAAAAAAAAAAA

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mag_scan
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  action_msg
  proto
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}  
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_proto
  SHARED
  proto/arguments.pb.cc 
  proto/arguments.pb.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
   obstacle_scan_node.cpp
   proto/arguments.pb.cc
   proto/arguments.pb.h)


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

