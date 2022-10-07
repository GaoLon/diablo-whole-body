cmake_minimum_required(VERSION 2.8.3)
project(ugv_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  robo_map
  front_end
  mpc
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS Eigen Boost
 CATKIN_DEPENDS robo_map mpc front_end
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_executable(ugv_planner_node
  src/ugv_planner_node.cpp 
  src/planner_manager.cpp
  src/traj_visualization.cpp
  src/minco_optimizer.cpp
  )

add_dependencies(ugv_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ugv_planner_node 
  ${catkin_LIBRARIES}
)

