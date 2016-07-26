cmake_minimum_required(VERSION 2.8.3)
project(trajectories)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  tf
  actionlib
  huskanypulator_msgs
  mbzirc_mission2_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectories
  CATKIN_DEPENDS 
    roscpp
    std_srvs
    tf
    actionlib
    huskanypulator_msgs
    mbzirc_mission2_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(trajectories
  src/Trajectories.cpp
)
add_dependencies(trajectories
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(trajectories
  ${catkin_LIBRARIES}
)

add_executable(trajectories_node
  src/trajectories_node.cpp
) 
add_dependencies(trajectories_node
  trajectories
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(trajectories_node
  trajectories
  ${catkin_LIBRARIES}
)