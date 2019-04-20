cmake_minimum_required(VERSION 2.8.3)
project(console_control)

find_package(catkin REQUIRED COMPONENTS
  hexapod_msgs
  message_runtime
  rospy
)

catkin_package(
  CATKIN_DEPENDS hexapod_msgs message_runtime rospy
)

catkin_python_setup()

catkin_install_python(PROGRAMS bin/main
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

