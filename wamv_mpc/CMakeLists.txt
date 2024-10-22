cmake_minimum_required(VERSION 3.0.2)
project(wamv_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
if (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "7.6") # Assuming GCC 4.9 supports C++14
    add_compile_options(-std=c++11)
else()
    add_compile_options(-std=c++14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  roslaunch
  tf
  nav_msgs
  geometry_msgs
  message_generation
  tf
  gazebo_msgs
  genmsg
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  )


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES wamv_mpc
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs gazebo_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(wamv_mpc_node 
  src/wamv_mpc_node.cpp
  src/wamv_mpc.cpp)
  
target_link_libraries(wamv_mpc_node
  ${catkin_LIBRARIES}
)
# add_dependencies(bluerov2_dob_node)

