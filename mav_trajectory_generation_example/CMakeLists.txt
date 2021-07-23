cmake_minimum_required(VERSION 2.8.3)
project(mav_trajectory_generation_example)

find_package(catkin_simple REQUIRED
nav_msgs
roscpp
rospy
std_msgs
geometry_msgs
visualization_msgs
trajectory_msgs
message_generation
core
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs core
)

catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

############
# BINARIES #
############

cs_add_executable(gate_minimum_snap
        src/minimum_snap_node.cpp
)

add_dependencies(gate_minimum_snap ${catkin_EXPORTED_TARGETS})

##########
# EXPORT #
##########
cs_install()
cs_export()
