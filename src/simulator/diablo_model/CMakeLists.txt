cmake_minimum_required(VERSION 2.8.3)
project(diablo_model)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs nav_msgs visualization_msgs
 DEPENDS Eigen
)

include_directories( 
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_executable (ugv_vis_node src/ugv_vis.cpp )
target_link_libraries(ugv_vis_node
  ${catkin_LIBRARIES})
