cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reachability_map)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_EXE_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-as-needed)
set(CMAKE_MODULE_LINKER_FLAGS -Wl,--no-as-needed)
find_package(catkin REQUIRED COMPONENTS
  exotica
  roscpp
  roslib
  drake_ik_solver
  message_generation
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
add_action_files(
  DIRECTORY action
  FILES DRM.action DRMTraj.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  exotica
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_reachability_map
  CATKIN_DEPENDS exotica roscpp actionlib_msgs message_generation roslib drake_ik_solver
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(dynamic_reachability_map
  src/dynamic_reachability_map/DRM.cpp
  src/dynamic_reachability_map/DRMSampler.cpp
  src/dynamic_reachability_map/DRMSpace.cpp
  src/dynamic_reachability_map/DRMSpaceSaver.cpp
  src/dynamic_reachability_map/DRMSpaceLoader.cpp
  src/dynamic_reachability_map/Conversions.cpp
  src/dynamic_reachability_map/DRMSampleCluster.cpp
  src/dynamic_reachability_map/DRMFullBodySampler.cpp
)
pods_use_pkg_config_packages(dynamic_reachability_map drake-rbm drake-ik)
target_link_libraries(dynamic_reachability_map 
${catkin_LIBRARIES} ${Boost_LIBRARIES} dynamic_reachability_map drakeRBM drakeIK
)

add_executable(DRMActionNode
  src/DRMActionNode.cpp
)
target_link_libraries(DRMActionNode 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} dynamic_reachability_map
)

add_executable(DRMSpaceSampler 
  src/drm_space_sampler.cpp 
)
pods_use_pkg_config_packages(DRMSpaceSampler drake-rbm drake-ik)
target_link_libraries(DRMSpaceSampler
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} dynamic_reachability_map drakeRBM drakeIK
)



