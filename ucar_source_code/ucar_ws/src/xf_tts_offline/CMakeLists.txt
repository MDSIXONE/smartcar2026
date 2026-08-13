cmake_minimum_required(VERSION 2.8.3)
project(xf_tts_offline)
add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(

)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

add_executable(xf_tts_node
  src/xf_tts_offline_node.cpp 
)

target_link_libraries(
   xf_tts_node
   ${catkin_LIBRARIES}
   -lmsc -lrt -ldl -lpthread 
 )

