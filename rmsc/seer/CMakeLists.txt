cmake_minimum_required(VERSION 2.8.3)
project(seer)

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
  actionlib_msgs 
  actionlib
  roslib
)

find_package(OpenCV REQUIRED)

# vvvvvvvvvvvvvvvvvvvvvvvvvv
find_package(Protobuf REQUIRED)
protobuf_generate_cpp(proto/arguments.pb.cc proto/arguments.pb.h proto/arguments.proto)

add_action_files(DIRECTORY action FILES Seer.action)
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)
#AAAAAAAAAAAAAAAAAAAAAAAAAAAAA


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mag_scan
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  proto
  action_msg
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}  
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_proto
  SHARED
  proto/arguments.pb.cc 
  proto/arguments.pb.h
)

add_executable(${PROJECT_NAME}_node 
   ${PROJECT_NAME}_node.cpp
   proto/arguments.pb.cc .
   proto/arguments.pb.h)


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

