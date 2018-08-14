project(rmsc_messages)

add_action_files(
  DIRECTORY action
  FILES
  GetAmmo.action
  NavTo.action
  LookAndMove.action
  AggressiveGainBuff.action
  ObstacleScan.action
  MapMux.action
  Seer.action
)


add_message_files(
  DIRECTORY msg
  FILES
  GripperCmd.msg
  GripperInfo.msg
  AggressiveGainBuffInfo.msg
  ConditionOverride.msg
  AmmoDetect.msg
)

generate_messages(
  DEPENDENCIES
  messages
  std_msgs
  geometry_msgs
  nav_msgs
)

set(rmsc_msg_INCLUDE_DIRS "${rmsc_messages_INCLUDE_DIRS}" CACHE  INTERNAL "rmsc messages Library location")