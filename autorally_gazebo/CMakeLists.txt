cmake_minimum_required(VERSION 2.8.3)
project(autorally_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  autorally_msgs
)

catkin_python_setup()

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    autorally_msgs
)

add_subdirectory(plugins)

install(PROGRAMS
  nodes/autorally_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

