cmake_minimum_required(VERSION 2.8.3)
project(hexapod_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  hexapod_msgs
  message_runtime
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  interface src/interface.cpp
)

target_link_libraries(
  interface ${catkin_LIBRARIES}
)
