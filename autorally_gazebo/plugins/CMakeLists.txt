cmake_minimum_required(VERSION 2.8.3)

find_package(gazebo REQUIRED)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED hello_world.cpp)
target_link_libraries(hello_world ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(spawn_obstacles SHARED spawn_obstacles.cpp)
target_link_libraries(spawn_obstacles ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})