cmake_minimum_required(VERSION 2.8.3)
project(front_end)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robo_map
  visualization_msgs
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES front_end
 CATKIN_DEPENDS roscpp rospy std_msgs robo_map visualization_msgs
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS}
)

add_library( front_end 
    src/ompl_planner_manager.cpp
    src/kino_astar.cpp
    )

target_link_libraries( front_end
    ${catkin_LIBRARIES} 
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_LIBRARIES}
    )
