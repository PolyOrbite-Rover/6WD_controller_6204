cmake_minimum_required(VERSION 2.8.3)
project(controller_6204)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  roscpp
  std_msgs
  nav_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package( INCLUDE_DIRS src 
	LIBRARIES controller_6204  
	CATKIN_DEPENDS pluginlib roscpp
  DEPENDS Boost
)

add_library( controller_6204_lib src/controller_6204.cpp )
target_link_libraries( controller_6204_lib ${catkin_LIBRARIES} )

