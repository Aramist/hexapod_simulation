cmake_minimum_required(VERSION 2.8.3)
project(hexapod_steps)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  hexapod_msgs
  message_runtime
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_steps
  CATKIN_DEPENDS hexapod_msgs message_runtime roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/hexapod_steps.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executables(${PROJECT_NAME}
  src/main.cc
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  launch/hexapod_stepping.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

