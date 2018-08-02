cmake_minimum_required(VERSION 2.8.3)
project(hexapod_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  Leg.msg
  HexapodState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

