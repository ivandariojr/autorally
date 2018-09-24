cmake_minimum_required(VERSION 2.8.3)

find_package(gazebo REQUIRED)
find_package(autorally_msgs REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_compile_options(-std=c++14)
add_library(obstacles_factory SHARED obstacles_factory.cpp)
target_link_libraries(obstacles_factory ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

if(gazebo_VERSION_MAJOR VERSION_EQUAL 9)
  target_compile_definitions(obstacles_factory PRIVATE GAZEBO_9)
endif()

add_dependencies(obstacles_factory autorally_msgs_gencpp)