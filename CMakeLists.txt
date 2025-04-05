cmake_minimum_required(VERSION 3.0.2)
project(final_project)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(mov_plugin SHARED src/mov_plugin.cc)
target_link_libraries(mov_plugin ${GAZEBO_LIBRARIES})

