cmake_minimum_required(VERSION 2.8.3)
project(robo_map)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES robo_map
 CATKIN_DEPENDS roscpp rospy std_msgs tf 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( robo_map 
    src/robo_map_manager.cpp
    )

target_link_libraries( robo_map
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )  
