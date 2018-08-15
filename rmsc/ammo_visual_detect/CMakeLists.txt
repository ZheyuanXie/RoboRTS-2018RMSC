project(ammo_visual_detect)

protobuf_generate_cpp(proto/arguments.pb.cc proto/arguments.pb.h proto/arguments.proto)

include_directories(
  proto
  ${Protobuf_INCLUDE_DIRS}  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ammo_visual_detect_proto
  SHARED
  proto/arguments.pb.cc 
  proto/arguments.pb.h
)

add_executable(ammo_visual_detect_node 
  mag_track_node.cpp 
  proto/arguments.pb.cc .
  proto/arguments.pb.h)

target_link_libraries(ammo_visual_detect_node
    ${PROTOBUF_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)



