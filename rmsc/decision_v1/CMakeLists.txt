project(rmsc_decision)

# protobuf

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto RMSCDecisionProtoSrc RMSCDecisionProtoHds ${ProtoFiles})

add_library(RMSC_behavior_tree_config
  SHARED
  ${RMSCDecisionProtoSrc}
  ${RMSCDecisionProtoHds}
)
target_link_libraries(RMSC_behavior_tree_config
  PUBLIC
  ${PROTOBUF_LIBRARIES}
)

# behavior tree

add_library(RMSC_behavior_tree INTERFACE)
target_sources(RMSC_behavior_tree
  INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_node.h
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_tree.h
  ${CMAKE_CURRENT_SOURCE_DIR}/blackboard.h)
target_link_libraries(RMSC_behavior_tree
  INTERFACE
  common::io
  common::log
  RMSC_behavior_tree_config
  ${catkin_LIBRARIES}
)

# decision node

add_executable(RMSC_decision_node_v1
  ${CMAKE_CURRENT_SOURCE_DIR}/decision_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/goal_factory.h
  ${CMAKE_CURRENT_SOURCE_DIR}/action_behavior.h
)

target_link_libraries(RMSC_decision_node_v1
  PRIVATE
  common::log
  RMSC_behavior_tree
  perception::map::costmap
  ${catkin_LIBRARIES}
)