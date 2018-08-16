project(rmsc_decision_final)

# protobuf

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto RMSCDecisionProtoSrc RMSCDecisionProtoHds ${ProtoFiles})

add_library(RMSC_behavior_tree_config_final
  SHARED
  ${RMSCDecisionProtoSrc}
  ${RMSCDecisionProtoHds}
)
target_link_libraries(RMSC_behavior_tree_config_final
  PUBLIC
  ${PROTOBUF_LIBRARIES}
)

# behavior tree

add_library(RMSC_behavior_tree_final INTERFACE)
target_sources(RMSC_behavior_tree_final
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

add_executable(RMSC_decision_node_v1_final
  ${CMAKE_CURRENT_SOURCE_DIR}/decision_node_final.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/goal_factory.h
  ${CMAKE_CURRENT_SOURCE_DIR}/action_behavior.h
)

target_link_libraries(RMSC_decision_node_v1_final
  PRIVATE
  common::log
  RMSC_behavior_tree
  perception::map::costmap
  ${catkin_LIBRARIES}
)

# add_executable(RMSC_decision_node_v2
#   ${CMAKE_CURRENT_SOURCE_DIR}/decision_node_final.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/goal_factory.h
#   ${CMAKE_CURRENT_SOURCE_DIR}/action_behavior.h
# )

# target_link_libraries(RMSC_decision_node_v2
#   PRIVATE
#   common::log
#   RMSC_behavior_tree
#   perception::map::costmap
#   ${catkin_LIBRARIES}
# )