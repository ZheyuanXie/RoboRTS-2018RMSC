project(ammo_visual_detect)

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
  ammo_visual_detect_node.cpp 
  proto/arguments.pb.cc .
  proto/arguments.pb.h)

target_link_libraries(${PROJECT_NAME}_node
    ${PROTOBUF_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_client
  ammo_visual_detect_client.cpp
)

target_link_libraries(${PROJECT_NAME}_client
  PRIVATE
  ${catkin_LIBRARIES}
)


