project(obstacle_scan)

protobuf_generate_cpp(proto/arguments.pb.cc proto/arguments.pb.h proto/arguments.proto)

include_directories(
  proto
  action_msg
  ${Protobuf_INCLUDE_DIRS}  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_proto
  SHARED
  proto/arguments.pb.cc 
  proto/arguments.pb.h
)

add_executable(${PROJECT_NAME}_node 
  obstacle_scan_node.cpp
  proto/arguments.pb.cc
  proto/arguments.pb.h)

target_link_libraries(${PROJECT_NAME}_node
  common::log
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

